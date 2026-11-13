
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Orbit;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoShoot;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.AutoConstants;


public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    // 機器人導向：推前 = 往車頭方向走，不依賴陀螺儀的「場地前方」校準
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(0.0).withRotationalDeadband(0.0) // 死區已在搖桿輸入端統一先處理，此處設為 0 避免雙重非線性阻尼
            .withDriveRequestType(DriveRequestType.Velocity); // 改用閉環速度控制模式，在阻力下維持穩定速度
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Shooter m_shooter = new Shooter();
    public final Hood m_hood = new Hood();
    public final Feeder m_feeder = new Feeder();
    public final Orbit m_orbit = new Orbit();
    public final Vision m_vision = new Vision();

    /** 起始邊（面向九宮格時機器人的左／右手邊）：+1 = 左起始框，-1 = 右起始框。 */
    private final SendableChooser<Double> m_startSide = new SendableChooser<>();

    public RobotContainer() {
        m_startSide.setDefaultOption("左起始框", 1.0);
        m_startSide.addOption("右起始框", -1.0);
        SmartDashboard.putData("Auto/Start Side", m_startSide);
        // 板子鎖定需要機器人位置與速度（場地座標）
        m_vision.setRobotState(() -> drivetrain.getState().Pose, () -> drivetrain.getState().Speeds);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                // 讀取搖桿輸入並先進行 10% 死區過濾 (WPILib慣例: 向前為 -Y, 向左為 -X, 逆時針旋轉為 -RightX)
                double xInput = MathUtil.applyDeadband(-joystick.getLeftY(), 0.1);
                double yInput = MathUtil.applyDeadband(-joystick.getLeftX(), 0.1);
                double rightX = MathUtil.applyDeadband(-joystick.getRightX(), 0.1);

                // 套用平方曲線以改善低速微調手感 (保留原本正負號)
                double xspeed = Math.copySign(xInput * xInput, xInput) * MaxSpeed;
                double yspeed = Math.copySign(yInput * yInput, yInput) * MaxSpeed;
                double rotRate = Math.copySign(rightX * rightX, rightX) * MaxAngularRate;

                // 實測：底盤設定的 +X 方向在實體上是「車右側」，所以操作上的「車頭」其實是設定的 +Y 方向。
                // 推前(xspeed) -> 往設定的 +Y 走；推左(yspeed) -> 往設定的 -X 走
                return drive.withVelocityX(-yspeed)
                            .withVelocityY(xspeed)
                            .withRotationalRate(rotRate); // Drive counterclockwise with negative X (left)
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // 【隔離 SysId 測試指令】避免與日常 X/Y 送球射擊功能衝突，並防止場上誤觸導致底盤暴衝
        // 若需要進行底盤系統辨識 (SysId Characterization)，請暫時解除註解並在安全開闊場地測試：
        /*
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // 當按住右板機 (Right Trigger) 時，以 60 RPS 基準速度、0.15 差速因子啟動發射器
        // 鬆開右板機時，呼叫停止發射器的命令
        joystick.rightTrigger().whileTrue(
            m_shooter.runShooterVelocityCommand(ShooterConstants.kShooterDefaultRPS, ShooterConstants.kShooterDefaultSpinFactor)
        ).onFalse(
            m_shooter.stopShooterCommand()
        );
        // 範例二：按住 X 鍵時，以開環百分比進行差速射擊
        // 基準電壓 70%，差速因子 0.2 (下輪輸出 84%，上輪輸出 56%)
        /*joystick.rightStick().whileTrue(
            m_shooter.runShooterDutyCycleCommand(0.7, 0.2)
        ).onFalse(
            m_shooter.stopShooterCommand()
        );*/

        // 按下 X 鍵瞬間（當射擊器轉速與仰角均就緒時）：分球盤步進 120°（Orbit 不動）
        joystick.x().and(m_shooter::isAtTargetVelocity)
                    .and(m_hood::isAtTargetPosition)
                    .onTrue(feedStepCommand());

        // 按住 Y 鍵：手動啟動導軌向前送球 (放開即停)
        joystick.y().whileTrue(m_orbit.feedCommand());

        // 十字鍵每按一下微調仰角 1 度：上 = +1°、下 = -1°（左右目前沒有功能）
        joystick.povUp().onTrue(m_hood.nudgeCommand(HoodConstants.kHoodNudgeRotations));
        joystick.povDown().onTrue(m_hood.nudgeCommand(-HoodConstants.kHoodNudgeRotations));

        // 按下右肩鍵 (Right Bumper)：手動單次分球，導軌同步送球 0.8 秒
        joystick.rightBumper().onTrue(feedStepCommand());

        // 按一下左板機 (Left Trigger)：Feeder 分球盤反向步進 120° 退球（Orbit 不動）
        joystick.leftTrigger().onTrue(unjamStepCommand());

        // 按下 Start 鍵：啟動視覺自主射擊（等同比賽前 60 秒的 AI 自主流程），最長 60 秒
        // 不依賴 Driver Station 的 Autonomous 模式，Teleop 模式下也能用。按 Back 鍵中止（排程一個佔用同樣機構的空指令，會取消正在跑的射擊流程）
        joystick.start().onTrue(autoShootCommand());
        joystick.back().onTrue(Commands.runOnce(() -> {}, drivetrain, m_hood, m_shooter, m_feeder, m_orbit));

        // 一鍵初始化：把仰角、分球盤「當下位置」都校準為 0 圈（Disabled 狀態也能按）
        // 用法：每次開電後，先用手把機構調到想要的起始角度，再「同時按下左右搖桿（L3 + R3）」
        // （自製儀表板是唯讀的看不到按鈕；SmartDashboard / Glass / Elastic 也有 "Init/Zero All" 按鈕）
        // 起始位置固定：同時把里程計設到儀表板選的起始框，場地圖上的位置才會對（板子鎖定也會清掉重來）
        final Command zeroAll = Commands.parallel(
                m_hood.zeroPositionCommand(),
                m_feeder.zeroPositionCommand(),
                Commands.runOnce(() -> {
                    drivetrain.resetPose(startPose());
                    m_vision.clearLock();
                })
            ).ignoringDisable(true);
        SmartDashboard.putData("Init/Zero All", zeroAll);
        joystick.leftStick().and(joystick.rightStick()).onTrue(zeroAll);
    }

    /**
     * 手動單次分球：只有 Feeder 步進 120°（Orbit 不動，Orbit 用 Y 鍵控制），之後再冷卻一小段時間。
     * 動作與冷卻期間再按鍵會被忽略（kCancelIncoming），避免連按導致一次轉超過 120°。
     */
    private Command feedStepCommand() {
        return m_feeder.stepForCommand()
                .andThen(Commands.waitSeconds(Constants.FeederConstants.kFeederCooldownSec))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    /**
     * 退球：只有 Feeder 反向步進 120°（Orbit 不動），之後冷卻。
     * 按一下只動一次，動作與冷卻期間再按會被忽略。
     */
    private Command unjamStepCommand() {
        return m_feeder.stepBackCommand()
                .andThen(Commands.waitSeconds(Constants.FeederConstants.kFeederCooldownSec))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    /**
     * 完整的前 60 秒自主流程（限時 AutoConstants.kAutoDurationSec 秒，移動時間也算在內）：
     *   1. 關掉相機（Orin 不拍照、不辨識），依儀表板選的起始邊把里程計設到該起始框
     *   2. 從起始框斜向直線開到射擊定點，車頭全程朝向九宮格
     *   3. 到位、車身停穩後才開相機，等視覺上線
     *   4. 視覺自主射擊（AutoShoot）：每個目標底盤原地轉向瞄準，到位後才送球
     * 起始邊在「指令開始的瞬間」才讀（defer），所以開賽前在儀表板改選擇都有效。
     */
    /** 儀表板選的起始框中心（場地座標），車頭朝九宮格。 */
    private Pose2d startPose() {
        final double side = m_startSide.getSelected() == null ? 1.0 : m_startSide.getSelected();
        return new Pose2d(AutoConstants.kStartCenterX, side * AutoConstants.kStartCenterAbsY, Rotation2d.kZero);
    }

    private Command autoShootCommand() {
        return Commands.defer(() -> {
            final Pose2d startPose = startPose();
            final Translation2d shootPoint = new Translation2d(AutoConstants.kShootX, AutoConstants.kShootY);

            return Commands.sequence(
                    Commands.runOnce(() -> {
                        m_vision.setCameraEnabled(false);
                        drivetrain.resetPose(startPose);
                        SmartDashboard.putString("Auto/Phase", "移動");
                    }),
                    new AutoDrive(drivetrain, shootPoint),
                    Commands.runOnce(() -> SmartDashboard.putString("Auto/Phase", "到位，等車身停穩")),
                    drivetrain.applyRequest(() -> brake).withTimeout(AutoConstants.kSettleSec),
                    Commands.runOnce(() -> {
                        m_vision.setCameraEnabled(true);
                        SmartDashboard.putString("Auto/Phase", "辨識與射擊");
                        SmartDashboard.putNumber("Auto/Distance To Board m", AutoConstants.kBoardX - drivetrain.getState().Pose.getX());
                    }),
                    // 射擊期間由 AutoShoot 控制底盤：每個目標原地轉向瞄準並保持（轉塔已拆除）
                    AutoShoot.routine(m_vision, drivetrain, m_hood, m_shooter, m_feeder, m_orbit))
                    // 不管是跑完、逾時或 Back 鍵中止，都把相機開回來（手動操作時儀表板才有畫面）
                    .finallyDo(() -> {
                        m_vision.setCameraEnabled(true);
                        SmartDashboard.putString("Auto/Phase", "結束");
                    });
        }, Set.of(drivetrain, m_hood, m_shooter, m_feeder, m_orbit))
                .withTimeout(AutoConstants.kAutoDurationSec)
                .withName("AutoMatch");
    }

    public Command getAutonomousCommand() {
        return autoShootCommand();
    }
}
