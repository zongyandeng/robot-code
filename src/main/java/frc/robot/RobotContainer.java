
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.Turret;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.HoodConstants;


public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // 新增 10% 死區限制
            .withDriveRequestType(DriveRequestType.Velocity); // 改用閉環速度控制模式，在阻力下維持穩定速度
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Shooter m_shooter = new Shooter();
    public final Turret m_turret = new Turret();
    public final Hood m_hood = new Hood();
    public final Feeder m_feeder = new Feeder();
    public final Orbit m_orbit = new Orbit();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                //讀取搖桿輸入(WPIlib中，向前為-Y，向左為-X，逆時針旋轉為-RightX)
                double xInput = -joystick.getLeftY();
                double yInput = -joystick.getLeftX();
                double rightX = -joystick.getRightX();
                //套用平方曲線以改善低速微調手感
                //copySign的作用是保留原本的前後左右方向(正負號)
                double xspeed = Math.copySign(xInput * xInput, xInput) * MaxSpeed;
                double yspeed = Math.copySign(yInput * yInput, yInput) * MaxSpeed;
                double rotRate = Math.copySign(rightX * rightX, rightX) * MaxAngularRate;

                return drive.withVelocityX(xspeed) // Drive forward with negative Y (forward)
                            .withVelocityY(yspeed) // Drive left with negative X (left)
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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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

        // 按下 X 鍵瞬間（當射擊器轉速、仰角與轉塔均就緒時）：分球盤步進，且導軌同時向前推進送球至發射器
        joystick.x().and(m_shooter::isAtTargetVelocity)
                    .and(m_hood::isAtTargetPosition)
                    .and(m_turret::isAtTargetPosition)
                    .onTrue(m_feeder.stepForCommand().alongWith(m_orbit.feedCommand().withTimeout(0.8)));

        // 按住 Y 鍵：手動啟動導軌向前送球 (放開即停)
        joystick.y().whileTrue(m_orbit.feedCommand());

        // 按下十字鍵上 : 仰角走到高位置(例如轉動0.1圈)
        joystick.povUp().onTrue(m_hood.goToPositionCommand(HoodConstants.kHoodPresetHighGoalRotations));

        // 按下十字鍵下 : 仰角降回初始位置 (0.0圈)
        joystick.povDown().onTrue(m_hood.goToPositionCommand(HoodConstants.kHoodStowedPositionRotations));

        // 按下十字鍵左 : 砲台偏向左側(例如0.1圈)
        joystick.povLeft().onTrue(m_turret.goToPositionCommand(-0.1));

        // 按下十字鍵右 : 砲台偏向右側(例如0.1圈)
        joystick.povRight().onTrue(m_turret.goToPositionCommand(0.1));

        // 按下右肩鍵 (Right Bumper)：手動單次分球，導軌同步送球 0.8 秒
        joystick.rightBumper().onTrue(m_feeder.stepForCommand().alongWith(m_orbit.feedCommand().withTimeout(0.8)));

        // 按住左板機 (Left Trigger)：Feeder 分球盤與 Orbit 導軌同時反轉退球（排障防卡球）
        joystick.leftTrigger().whileTrue(m_feeder.reverseCommand().alongWith(m_orbit.reverseCommand()));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
