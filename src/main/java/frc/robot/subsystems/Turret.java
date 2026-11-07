package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase{
    private final TalonFX turret_motor = new TalonFX(TurretConstants.kTurretId);
    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(TurretConstants.kTurretMotionMagicVoltage);

    private double m_targetPosition = 0.0; // 目標位置 (圈數)

    public Turret() {
        var config = new TalonFXConfiguration();
        
        config.Feedback.SensorToMechanismRatio = TurretConstants.kTurretGearRatio; // 設定馬達編碼器與轉塔機構的齒輪比

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // 設定馬達空轉時為煞車模式 (Brake Mode)，防止轉塔自由旋轉
        config.MotorOutput.Inverted = TurretConstants.kTurretMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // 設定馬達旋轉方向是否反向
        // 1. 位置控制 PID 參數設定
        config.Slot0.kP = TurretConstants.kTurretkP; 
        config.Slot0.kI = TurretConstants.kTurretkI; 
        config.Slot0.kD = TurretConstants.kTurretkD; 
        config.Slot0.kV = TurretConstants.kTurretkV;
        config.Slot0.kS = TurretConstants.kTurretkS;

        // 2. 啟用並設定左右軟體限位（正負 90 度對應機構端 ±0.25 圈，已配置 SensorToMechanismRatio 故單位為機構端圈數）        
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = TurretConstants.kTurretForwardSoftLimitEnable;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.kTurretForwardSoftLimitThreshold;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = TurretConstants.kTurretReverseSoftLimitEnable;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.kTurretReverseSoftLimitThreshold;

        // 3. Motion Magic 運動軌跡控制參數設定（限制最大速度與最大加速度，防機構暴衝）
        config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.kTurretMotionMagicCruiseVelocity; 
        config.MotionMagic.MotionMagicAcceleration = TurretConstants.kTurretMotionMagicAcceleration;
        config.MotionMagic.MotionMagicJerk = TurretConstants.kTurretMotionMagicJerk; 

        // 4. 定子電流限制設定（Stator Current Limit）：防止物理卡死撞擊時燒毀馬達與損壞齒輪
        config.CurrentLimits.StatorCurrentLimit = TurretConstants.kTurretStatorCurrentLimit; 
        config.CurrentLimits.StatorCurrentLimitEnable = TurretConstants.kTurretStatorCurrentLimitEnable;

        // 5. 電源端電流限制設定（Supply Current Limit）：防止馬達瞬間抽電過大導致電池掉壓 (Brownout)
        config.CurrentLimits.SupplyCurrentLimit = TurretConstants.kTurretSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = TurretConstants.kTurretSupplyCurrentLowerLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = TurretConstants.kTurretSupplyCurrentLowerTime;
        config.CurrentLimits.SupplyCurrentLimitEnable = TurretConstants.kTurretSupplyCurrentLimitEnable;

        turret_motor.getConfigurator().apply(config);
        turret_motor.setPosition(0.0); // 開電初始化自動將當前位置校準為 0 圈

        // 在 SmartDashboard 提供一鍵歸零按鈕 (支援在 Disabled 狀態下推正機構後手動歸零)
        SmartDashboard.putData("Turret/Zero Position", zeroPositionCommand().ignoringDisable(true));

    }

    /**
     * 手動將當前轉塔位置重新校準歸零為 0 圈
     */
    public Command zeroPositionCommand() {
        return this.runOnce(() -> {
            turret_motor.setPosition(0.0);
            m_targetPosition = 0.0;
        });
    }

    /**
     * 讓砲台選轉到目標角度
     * @param targetRotation 目標轉圈數 (例如 : 轉1.5圈，配合齒輪比即為特定角度)
     */
    public Command goToPositionCommand(double targetRotation) {
        return this.run(() -> {
            m_targetPosition = targetRotation;
            turret_motor.setControl(m_motionMagicVoltage.withPosition(m_targetPosition));
        });
    }
    public boolean isAtTargetPosition() {
        double currentPosition = turret_motor.getPosition().getValueAsDouble();
        return Math.abs(currentPosition - m_targetPosition) < TurretConstants.kTurretPositionToleranceRotations;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Position (Rotations)", turret_motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret/Position (Degrees)", turret_motor.getPosition().getValueAsDouble() * 360.0);
        SmartDashboard.putBoolean("Turret/At Target", isAtTargetPosition());
    }    
}