package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.ShotTable;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Orbit;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * 視覺自主射擊流程（機器人停在定點，只靠轉塔、仰角、發射輪瞄準，底盤不動）：
 *   1. 等視覺上線
 *   2. 先打亮燈的小孔 (kShotsAtLitSmallHole 顆)
 *   3. 依 kBigHoleOrder 逐個大孔各打 kShotsPerBigHole 顆，直到 27 顆用完
 *
 * 每個目標一個「段落」：用視覺中位數量測算出轉塔圈數、仰角、轉速，三個機構一起走到位，
 * 並在到位後才送球（沿用手動時的 Feeder 步進 + Orbit 送球）。到位逾時就不發射、不消耗球數。
 * 轉塔 / 仰角 / 發射輪在同一個目標的連發之間保持運轉。
 */
public final class AutoShoot {
    private AutoShoot() {}

    public static Command routine(Vision vision, Turret turret, Hood hood, Shooter shooter, Feeder feeder, Orbit orbit) {
        final int[] fired = {0};
        final Set<Subsystem> reqs = Set.of(turret, hood, shooter, feeder, orbit);

        List<Command> steps = new ArrayList<>();
        steps.add(Commands.runOnce(() -> fired[0] = 0));
        steps.add(Commands.waitUntil(vision::isConnected).withTimeout(AutoConstants.kWaitForVisionSec));

        // 亮燈小孔：先等燈號穩定（最多 2 秒），仍認不出來就略過這一段
        steps.add(Commands.waitUntil(() -> vision.getLitSmallCol() >= 0).withTimeout(2.0));
        steps.add(Commands.defer(() -> {
            int col = vision.getLitSmallCol();
            if (col < 0) {
                DriverStation.reportWarning("AutoShoot: 認不出亮燈的小孔，略過小孔段落", false);
                return Commands.none();
            }
            return shootAt(col, AutoConstants.kShotsAtLitSmallHole, vision, turret, hood, shooter, feeder, orbit, fired);
        }, reqs));

        for (int index : AutoConstants.kBigHoleOrder) {
            steps.add(shootAt(index, AutoConstants.kShotsPerBigHole, vision, turret, hood, shooter, feeder, orbit, fired));
        }

        return Commands.sequence(steps.toArray(new Command[0]))
                .finallyDo(() -> SmartDashboard.putNumber("AutoShoot/Balls Fired", fired[0]))
                .withName("AutoShoot");
    }

    /** 對指定孔 (0~11) 發射 shots 顆。目標在段落開始的瞬間才向視覺取值。 */
    private static Command shootAt(int index, int shots, Vision vision, Turret turret, Hood hood, Shooter shooter,
            Feeder feeder, Orbit orbit, int[] fired) {
        final Set<Subsystem> reqs = Set.of(turret, hood, shooter, feeder, orbit);
        return Commands.defer(() -> {
            if (fired[0] >= AutoConstants.kMaxBalls) {
                return Commands.none();
            }
            Optional<Vision.Target> t = vision.getTarget(index);
            if (t.isEmpty()) {
                DriverStation.reportWarning("AutoShoot: 孔 " + index + " 沒有有效的視覺資料，略過", false);
                return Commands.none();
            }
            Vision.Target target = t.get();
            if (Double.isNaN(target.distM())) {
                DriverStation.reportWarning("AutoShoot: 孔 " + index + " 沒有深度資料，無法查表，略過", false);
                return Commands.none();
            }

            double turretRot = turretRotations(target);
            ShotTable.Shot shot = ShotTable.lookup(rowOf(index), target.distM());
            SmartDashboard.putNumber("AutoShoot/Target Index", index);
            SmartDashboard.putNumber("AutoShoot/Turret Rot", turretRot);
            SmartDashboard.putNumber("AutoShoot/Hood Rot", shot.hoodRotations());
            SmartDashboard.putNumber("AutoShoot/Shooter RPS", shot.shooterRps());
            SmartDashboard.putNumber("AutoShoot/Distance m", target.distM());

            // 到位後才發射；逾時就不發、不扣球數（每一發要建立新的指令物件，WPILib 不允許同一個指令放進組合兩次）
            Supplier<Command> oneShot = () -> Commands.sequence(
                    Commands.waitUntil(() -> shooter.isAtTargetVelocity() && hood.isAtTargetPosition() && turret.isAtTargetPosition())
                            .withTimeout(AutoConstants.kReadyTimeoutSec),
                    Commands.either(
                            fire(feeder, orbit, fired),
                            Commands.runOnce(() -> DriverStation.reportWarning("AutoShoot: 機構未到位，這一發不射", false)),
                            () -> shooter.isAtTargetVelocity() && hood.isAtTargetPosition() && turret.isAtTargetPosition()
                                    && fired[0] < AutoConstants.kMaxBalls));

            List<Command> seq = new ArrayList<>();
            seq.add(Commands.waitSeconds(AutoConstants.kPrepWaitSec)); // 讓三個機構先收到新目標，再去檢查「到位」
            for (int i = 0; i < shots; i++) {
                seq.add(oneShot.get());
            }
            // sequence 結束 = 這個目標打完；三個機構的「走到目標」指令跟著被取消
            return Commands.deadline(
                    Commands.sequence(seq.toArray(new Command[0])),
                    turret.goToPositionCommand(turretRot),
                    hood.goToPositionCommand(shot.hoodRotations()),
                    shooter.runShooterVelocityCommand(shot.shooterRps(), ShooterConstants.kShooterDefaultSpinFactor));
        }, reqs);
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

    /**
     * 把「相機看到的目標」換算成轉塔馬達圈數。相機裝在底盤上（不隨轉塔轉）：
     * 先把目標放進機器人座標（前、左），再扣掉相機相對轉塔軸的偏移，求出轉塔軸看目標往右幾度。
     */
    static double turretRotations(Vision.Target t) {
        double b = Math.toRadians(t.bearingDeg());          // 相機看目標，往右為正
        double d = Double.isNaN(t.distM()) ? 0.0 : t.distM();
        double forward = VisionConstants.kCameraForwardOfTurretMeters + d * Math.cos(b);
        double left = VisionConstants.kCameraLeftOfTurretMeters - d * Math.sin(b);
        double angleRightDeg = Double.isNaN(t.distM())
                ? t.bearingDeg()                              // 沒有距離就不做視差修正
                : Math.toDegrees(Math.atan2(-left, forward));
        double rot = VisionConstants.kTurretDirection * angleRightDeg / 360.0 + VisionConstants.kTurretZeroOffsetRot;
        double lim = VisionConstants.kTurretSafeLimitRot;
        return Math.max(-lim, Math.min(lim, rot));
    }
}
