package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * 自主開場：從起始框「斜向」直線開到射擊定點，車頭全程保持朝向九宮格（場地座標的 +X）。
 *
 * 座標系是 RobotContainer 自己定義的「場地座標」（不是 FRC 官方場地）：
 *   原點 = 機器人作業區後緣中心，+X 朝九宮格，+Y 朝左（面向九宮格時的左手邊），車頭朝 0 度。
 * 里程計由 RobotContainer 在開跑前用 resetPose() 設成起始框位置。
 *
 * 用 ApplyFieldSpeeds（固定以藍方視角的場地座標運算，不受 Driver Station 的聯盟顏色影響）。
 * 速度規劃：加速有斜率限制、接近目標時依 sqrt(2*a*d) 提前減速，到達容許誤差內就結束。
 * 這個指令只負責「開過去」，到位後的煞車由呼叫端接手。
 */
public final class AutoDrive extends Command {
    private final CommandSwerveDrivetrain m_drive;
    private final Translation2d m_target;
    private final SwerveRequest.ApplyFieldSpeeds m_request = new SwerveRequest.ApplyFieldSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);
    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(AutoConstants.kDriveMaxAccelMps2);
    private final Timer m_timer = new Timer();

    public AutoDrive(CommandSwerveDrivetrain drive, Translation2d target) {
        m_drive = drive;
        m_target = target;
        addRequirements(drive);
        setName("AutoDrive");
    }

    @Override
    public void initialize() {
        m_speedLimiter.reset(0.0);
        m_timer.restart();
    }

    @Override
    public void execute() {
        Pose2d pose = m_drive.getState().Pose;
        Translation2d delta = m_target.minus(pose.getTranslation());
        double dist = delta.getNorm();

        // 目標速度：不超過上限，接近終點時依減速度提前降速（保留一點最低速度，才不會停在容許誤差外）
        double desired = Math.min(AutoConstants.kDriveMaxSpeedMps,
                Math.sqrt(2.0 * AutoConstants.kDriveMaxAccelMps2 * dist));
        desired = Math.max(desired, AutoConstants.kDriveMinSpeedMps);
        double speed = m_speedLimiter.calculate(desired);

        double vx = 0.0;
        double vy = 0.0;
        if (dist > 1e-6) {
            vx = speed * delta.getX() / dist;
            vy = speed * delta.getY() / dist;
        }

        // 相機那面維持朝向九宮格（場地 0 度）= 底盤車頭 -kCameraYawOffsetDeg
        double headingErr = MathUtil.angleModulus(
                -Math.toRadians(VisionConstants.kCameraYawOffsetDeg) - pose.getRotation().getRadians());
        double omega = MathUtil.clamp(AutoConstants.kDriveHeadingKp * headingErr,
                -AutoConstants.kDriveMaxOmegaRadPerSec, AutoConstants.kDriveMaxOmegaRadPerSec);

        m_drive.setControl(m_request.withSpeeds(new ChassisSpeeds(vx, vy, omega)));

        SmartDashboard.putNumber("AutoDrive/Distance To Target m", dist);
        SmartDashboard.putNumber("AutoDrive/Pose X", pose.getX());
        SmartDashboard.putNumber("AutoDrive/Pose Y", pose.getY());
        SmartDashboard.putNumber("AutoDrive/Heading deg", pose.getRotation().getDegrees());
    }

    @Override
    public boolean isFinished() {
        double dist = m_target.getDistance(m_drive.getState().Pose.getTranslation());
        if (dist < AutoConstants.kDriveToleranceMeters) {
            return true;
        }
        if (m_timer.hasElapsed(AutoConstants.kDriveTimeoutSec)) {
            DriverStation.reportWarning("AutoDrive: 逾時，離定點還差 " + String.format("%.2f", dist) + " m，直接開始射擊", false);
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // 停住；之後由呼叫端的煞車指令接手（被中止時 Back 鍵的空指令會佔住底盤）
        m_drive.setControl(m_request.withSpeeds(new ChassisSpeeds()));
    }
}
