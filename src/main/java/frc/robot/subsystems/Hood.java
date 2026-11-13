package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

    private final TalonFX hood_motor = new TalonFX(HoodConstants.kHoodId);

    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(HoodConstants.kHoodMotionMagicVoltage);

    private double m_targetPosition = HoodConstants.kHoodStartPositionRotations; // 目標位置 (圈數)，開電時在最高角度

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
        hood_motor.setPosition(HoodConstants.kHoodStartPositionRotations); // 開電初始化：把當前位置視為最高角度

        // 在 SmartDashboard 提供一鍵歸零按鈕 (支援在 Disabled 狀態下推正機構後手動歸零)
        SmartDashboard.putData("Hood/Zero Position", zeroPositionCommand().ignoringDisable(true));
   
    }

    /**
     * 手動將當前仰角位置重新校準歸零為 0 圈 (收納位置)
     */
    public Command zeroPositionCommand() {
        return this.runOnce(() -> {
            hood_motor.setPosition(HoodConstants.kHoodStartPositionRotations);
            m_targetPosition = HoodConstants.kHoodStartPositionRotations;
            hood_motor.setControl(m_motionMagicVoltage.withPosition(m_targetPosition)); // 讓馬達的目標也回到起始位置，避免啟用時衝回舊目標
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

    /**
     * 相對微調：以目前目標位置為基準，增減指定圈數（限制在軟體限位內）
     * @param deltaRotations 要增減的機構端圈數（1 度 = 1/360 圈）
     */
    public Command nudgeCommand(double deltaRotations) {
        return this.runOnce(() -> {
            m_targetPosition = Math.max(HoodConstants.kHoodReverseSoftLimitThreshold,
                    Math.min(HoodConstants.kHoodForwardSoftLimitThreshold, m_targetPosition + deltaRotations));
            hood_motor.setControl(m_motionMagicVoltage.withPosition(m_targetPosition));
        })
        // 動作後冷卻一段時間，冷卻期間再按鍵會被忽略（kCancelIncoming）
        .andThen(Commands.waitSeconds(HoodConstants.kHoodNudgeCooldownSec))
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    public boolean isAtTargetPosition() {
        double currentPos = hood_motor.getPosition().getValueAsDouble();
        return Math.abs(currentPos - m_targetPosition) <= HoodConstants.kHoodPositionToleranceRotations;
    }

    @Override
    public void periodic() {
        double currentRot = hood_motor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Hood/Position (Rotations)", currentRot);
        SmartDashboard.putNumber("Hood/Target (Rotations)", m_targetPosition);
        SmartDashboard.putNumber("Hood/Motor Voltage (V)", hood_motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Position (Degrees)", currentRot * 360.0);
        SmartDashboard.putBoolean("Hood/At Target", isAtTargetPosition());
    }    
}
