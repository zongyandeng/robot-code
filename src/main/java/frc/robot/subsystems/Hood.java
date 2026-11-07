package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

    private final TalonFX hood_motor = new TalonFX(HoodConstants.kHoodId);

    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(HoodConstants.kHoodMotionMagicVoltage);

    private double m_targetPosition = HoodConstants.kHoodStowedPositionRotations; // 目標位置 (圈數)

    public Hood() {
        var config = new TalonFXConfiguration();

        config.Feedback.SensorToMechanismRatio = HoodConstants.kHoodGearRatio; // 設定馬達編碼器與仰角機構的齒輪比
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // 設定馬達空轉時為煞車模式 (Brake Mode)，防止仰角自由旋轉
        
        // 1. 位置控制 PID 參數設定
        config.Slot0.kP = HoodConstants.kHoodkP;
        config.Slot0.kI = HoodConstants.kHoodkI;
        config.Slot0.kD = HoodConstants.kHoodkD; 
        config.Slot0.kV = HoodConstants.kHoodkV;
        config.Slot0.kS = HoodConstants.kHoodkS;

        // 2. 啟用並設定軟體限位（限制在水平 0 度到向上 90 度之間的圈數，已乘上齒輪比）
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = HoodConstants.kHoodForwardSoftLimitEnable;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.kHoodForwardSoftLimitThreshold; 
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = HoodConstants.kHoodReverseSoftLimitEnable;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.kHoodReverseSoftLimitThreshold;

        // 3. Motion Magic 運動軌跡控制參數設定（防仰角面板升降時猛烈撞擊）
        config.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.kHoodMotionMagicCruiseVelocity; 
        config.MotionMagic.MotionMagicAcceleration = HoodConstants.kHoodMotionMagicAcceleration; 
        config.MotionMagic.MotionMagicJerk = HoodConstants.kHoodMotionMagicJerk;

        // 4. 定子電流限制設定（Stator Current Limit）：防止物理卡死時頂壞連桿與機構
        config.CurrentLimits.StatorCurrentLimit = HoodConstants.kHoodStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = HoodConstants.kHoodStatorCurrentLimitEnable;

        // 5. 電源端電流限制設定（Supply Current Limit）：防止加速抽電導致電壓驟降 (Brownout)
        config.CurrentLimits.SupplyCurrentLimit = HoodConstants.kHoodSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = HoodConstants.kHoodSupplyCurrentLowerLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = HoodConstants.kHoodSupplyCurrentLowerTime;
        config.CurrentLimits.SupplyCurrentLimitEnable = HoodConstants.kHoodSupplyCurrentLimitEnable;
        
        hood_motor.getConfigurator().apply(config);
        hood_motor.setPosition(0.0); // 開電初始化自動將當前位置校準為 0 圈 (收納位置)

        // 在 SmartDashboard 提供一鍵歸零按鈕 (支援在 Disabled 狀態下推正機構後手動歸零)
        SmartDashboard.putData("Hood/Zero Position", zeroPositionCommand().ignoringDisable(true));
   
    }

    /**
     * 手動將當前仰角位置重新校準歸零為 0 圈 (收納位置)
     */
    public Command zeroPositionCommand() {
        return this.runOnce(() -> {
            hood_motor.setPosition(0.0);
            m_targetPosition = 0.0;
        });
    }

    /** 
     * 讓砲台旋轉到目標角度
     * @param targetRotations 目標轉圈數（例如：轉 1.5 圈，配合齒輪比即為特定角度）
     */
    public Command goToPositionCommand(double targetRotation){
        // 命令馬達走到目標圈數
        return this.run(() -> {
            m_targetPosition = targetRotation;
            hood_motor.setControl(m_motionMagicVoltage.withPosition(targetRotation));
        });
    }

    public boolean isAtTargetPosition() {
        double currentPos = hood_motor.getPosition().getValueAsDouble();
        return Math.abs(currentPos - m_targetPosition) <= HoodConstants.kHoodPositionToleranceRotations;
    }

    @Override
    public void periodic() {
        double currentRot = hood_motor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Hood/Position (Rotations)", currentRot);
        SmartDashboard.putNumber("Hood/Position (Degrees)", currentRot * 360.0);
        SmartDashboard.putBoolean("Hood/At Target", isAtTargetPosition());
    }    
}
