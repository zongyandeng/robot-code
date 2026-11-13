package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.ShotTable;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Orbit;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * 視覺自主射擊流程（機器人停在定點；轉塔已拆除，左右瞄準改由底盤原地轉向）：
 *   1. 等視覺上線
 *   2. 先打亮燈的小孔 (kShotsAtLitSmallHole 顆)
 *   3. 依 kBigHoleOrder 逐個大孔各打 kShotsPerBigHole 顆，直到 27 顆用完
 *
 * 視覺上線後先等「板子鎖定」（看到全部 12 孔時把孔換成場地座標記住，見 Vision）最多 kWaitForLockSec 秒。
 * 每個目標一個「段落」：段落開始時決定車頭角度與距離，
 *   - 有鎖定：用目前里程計位置對鎖定的孔位置計算，並考慮發射口在車身左前方的偏移（之後孔出了畫面也能瞄）；
 *   - 沒鎖定：用即時視覺的中位數，車頭目標角度 = 目前車頭角度 - 偏角 (bearing，往右為正)；
 *     距離用視覺的 dist_m，沒有就用里程計推算 (kBoardX - 目前 X) / cos(偏角)。
 *   - 兩者都再扣 kAimBiasDeg，底盤原地轉過去並保持；用距離查 ShotTable 得到仰角與轉速。
 * 車頭、仰角、發射輪都到位後才送球（Feeder 步進 + Orbit 送球）。到位逾時就不發射、不消耗球數。
 * 同一個目標的連發之間，三者都保持在目標上。
 */
public final class AutoShoot {
    private AutoShoot() {}

    public static Command routine(Vision vision, CommandSwerveDrivetrain drive, Hood hood, Shooter shooter,
            Feeder feeder, Orbit orbit) {
        final int[] fired = {0};
        final Set<Subsystem> reqs = Set.of(drive, hood, shooter, feeder, orbit);

        List<Command> steps = new ArrayList<>();
        steps.add(Commands.runOnce(() -> fired[0] = 0));
        steps.add(Commands.waitUntil(vision::isConnected).withTimeout(AutoConstants.kWaitForVisionSec));
        // 等看到全部孔、鎖定板位（之後仰角機構把相機帶高、孔出了畫面也能瞄）；逾時就用即時視覺
        steps.add(Commands.waitUntil(vision::isLocked).withTimeout(AutoConstants.kWaitForLockSec));
        steps.add(Commands.runOnce(() -> SmartDashboard.putString("AutoShoot/Aim Source",
                vision.isLocked() ? "鎖定板位" : "即時視覺（沒鎖定）")));

        // 亮燈小孔：先等燈號穩定（最多 2 秒），仍認不出來就略過這一段
        steps.add(Commands.waitUntil(() -> vision.getLitSmallCol() >= 0).withTimeout(2.0));
        steps.add(Commands.defer(() -> {
            int col = vision.getLitSmallCol();
            if (col < 0) {
                DriverStation.reportWarning("AutoShoot: 認不出亮燈的小孔，略過小孔段落", false);
                return Commands.none();
            }
            return shootAt(col, AutoConstants.kShotsAtLitSmallHole, vision, drive, hood, shooter, feeder, orbit, fired);
        }, reqs));

        for (int index : AutoConstants.kBigHoleOrder) {
            steps.add(shootAt(index, AutoConstants.kShotsPerBigHole, vision, drive, hood, shooter, feeder, orbit, fired));
        }

        return Commands.sequence(steps.toArray(new Command[0]))
                .finallyDo(() -> SmartDashboard.putNumber("AutoShoot/Balls Fired", fired[0]))
                .withName("AutoShoot");
    }

    /** 對指定孔 (0~11) 發射 shots 顆。目標在段落開始的瞬間才向視覺取值。 */
    private static Command shootAt(int index, int shots, Vision vision, CommandSwerveDrivetrain drive, Hood hood,
            Shooter shooter, Feeder feeder, Orbit orbit, int[] fired) {
        final Set<Subsystem> reqs = Set.of(drive, hood, shooter, feeder, orbit);
        return Commands.defer(() -> {
            if (fired[0] >= AutoConstants.kMaxBalls) {
                return Commands.none();
            }
            Pose2d pose = drive.getState().Pose;
            double headingNow = pose.getRotation().getRadians();
            double headingTarget;
            double distM;
            String distSource;
            Optional<Vision.Aim> lock = vision.aimFromLock(index, pose);
            if (lock.isPresent()) {
                // 鎖定板位：已含發射口偏移；不需要孔在畫面裡
                headingTarget = MathUtil.angleModulus(lock.get().headingRad() - Math.toRadians(AutoConstants.kAimBiasDeg));
                distM = lock.get().distM();
                distSource = "鎖定板位";
            } else {
                Optional<Vision.Target> t = vision.getTarget(index);
                if (t.isEmpty()) {
                    DriverStation.reportWarning("AutoShoot: 孔 " + index + " 沒有鎖定也沒有有效的視覺資料，略過", false);
                    return Commands.none();
                }
                Vision.Target target = t.get();
                // 車頭目標角度：偏角往右為正，WPILib 的角度逆時針為正，所以往右轉 = 角度減少
                headingTarget = MathUtil.angleModulus(
                        headingNow - Math.toRadians(target.bearingDeg() + AutoConstants.kAimBiasDeg));
                // 距離：優先用視覺（solvePnP）；沒有就用里程計推算到板面的距離，再依偏角換成到該孔的水平距離
                distM = target.distM();
                distSource = "即時視覺";
                if (Double.isNaN(distM)) {
                    double toBoard = AutoConstants.kBoardX - pose.getX();
                    distM = toBoard / Math.max(0.5, Math.cos(Math.toRadians(target.bearingDeg())));
                    distSource = "里程計推算";
                }
            }

            ShotTable.Shot shot = ShotTable.lookup(rowOf(index), distM);
            SmartDashboard.putNumber("AutoShoot/Target Index", index);
            SmartDashboard.putNumber("AutoShoot/Hood Rot", shot.hoodRotations());
            SmartDashboard.putNumber("AutoShoot/Shooter RPS", shot.shooterRps());
            SmartDashboard.putNumber("AutoShoot/Distance m", distM);
            SmartDashboard.putString("AutoShoot/Distance Source", distSource);
            SmartDashboard.putNumber("AutoShoot/Bearing deg", Math.toDegrees(MathUtil.angleModulus(headingNow - headingTarget)));
            SmartDashboard.putNumber("AutoShoot/Heading Target deg", Math.toDegrees(headingTarget));

            BooleanSupplier aimed = () -> Math.abs(headingError(drive, headingTarget))
                    < Math.toRadians(AutoConstants.kAimToleranceDeg);
            BooleanSupplier ready = () -> shooter.isAtTargetVelocity() && hood.isAtTargetPosition()
                    && aimed.getAsBoolean();

            // 到位後才發射；逾時就不發、不扣球數（每一發要建立新的指令物件，WPILib 不允許同一個指令放進組合兩次）
            Supplier<Command> oneShot = () -> Commands.sequence(
                    Commands.waitUntil(ready).withTimeout(AutoConstants.kReadyTimeoutSec),
                    Commands.either(
                            fire(feeder, orbit, fired),
                            Commands.runOnce(() -> DriverStation.reportWarning(
                                    "AutoShoot: 未到位（車頭/仰角/轉速），這一發不射", false)),
                            () -> ready.getAsBoolean() && fired[0] < AutoConstants.kMaxBalls));

            List<Command> seq = new ArrayList<>();
            seq.add(Commands.waitSeconds(AutoConstants.kPrepWaitSec)); // 讓機構先收到新目標，再去檢查「到位」
            for (int i = 0; i < shots; i++) {
                seq.add(oneShot.get());
            }
            // sequence 結束 = 這個目標打完；車頭、仰角、發射輪的「走到目標」指令跟著被取消
            return Commands.deadline(
                    Commands.sequence(seq.toArray(new Command[0])),
                    aimCommand(drive, headingTarget),
                    hood.goToPositionCommand(shot.hoodRotations()),
                    shooter.runShooterVelocityCommand(shot.shooterRps(), ShooterConstants.kShooterDefaultSpinFactor));
        }, reqs);
    }

    /** 底盤原地轉到 headingTarget 並保持；結束時（被取消）停止轉動。 */
    private static Command aimCommand(CommandSwerveDrivetrain drive, double headingTarget) {
        final SwerveRequest.ApplyFieldSpeeds request = new SwerveRequest.ApplyFieldSpeeds()
                .withDriveRequestType(DriveRequestType.Velocity);
        final double tol = Math.toRadians(AutoConstants.kAimToleranceDeg);
        return drive.runEnd(() -> {
            double err = headingError(drive, headingTarget);
            double omega = 0.0;
            if (Math.abs(err) > tol * 0.5) {
                omega = AutoConstants.kAimKp * err + Math.copySign(AutoConstants.kAimMinOmegaRadPerSec, err);
                omega = MathUtil.clamp(omega, -AutoConstants.kAimMaxOmegaRadPerSec, AutoConstants.kAimMaxOmegaRadPerSec);
            }
            drive.setControl(request.withSpeeds(new ChassisSpeeds(0.0, 0.0, omega)));
            SmartDashboard.putNumber("AutoShoot/Heading Error deg", Math.toDegrees(err));
        }, () -> drive.setControl(request.withSpeeds(new ChassisSpeeds())))
                .withName("AutoShoot/Aim");
    }

    /** 目標角度減目前角度，包在 -pi ~ pi（正 = 要逆時針轉）。 */
    private static double headingError(CommandSwerveDrivetrain drive, double headingTarget) {
        return MathUtil.angleModulus(headingTarget - drive.getState().Pose.getRotation().getRadians());
    }

    private static Command fire(Feeder feeder, Orbit orbit, int[] fired) {
        return Commands.sequence(
                feeder.stepForCommand().alongWith(orbit.feedCommand().withTimeout(AutoConstants.kFeedSec)),
                Commands.runOnce(() -> {
                    fired[0]++;
                    SmartDashboard.putNumber("AutoShoot/Balls Fired", fired[0]);
                }),
                Commands.waitSeconds(AutoConstants.kCooldownSec));
    }

    /** index 0~2 為小孔排 (row 0)；3~5、6~8、9~11 依序為 row 1~3。 */
    static int rowOf(int index) {
        return index / 3;
    }

}
