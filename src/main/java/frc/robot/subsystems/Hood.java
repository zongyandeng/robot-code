package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

    private final TalonFX hood_motor = new TalonFX(HoodConstants.kHoodId);
    // 建立一個位置控制請求物件
    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(HoodConstants.kHoodMotionMagicVoltage);

    public Hood() {
        // 設定馬達的 PID 參數，讓馬達知道如何自我微調、不衝過頭
        var config = new TalonFXConfiguration();
        config.Slot0.kP = HoodConstants.kHoodkP; // 比例參數：數值越大，馬達反應越快，但太大會抖動
        config.Slot0.kI = HoodConstants.kHoodkI; //機分參數
        config.Slot0.kD = HoodConstants.kHoodkD; // 微分參數：用來減速防震，防止馬達衝過頭

        config.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.kHoodMotionMagicCruiseVelocity; //最大速度(圈/秒)
        config.MotionMagic.MotionMagicAcceleration = HoodConstants.kHoodMotionMagicAcceleration; //最大加速度
        config.MotionMagic.MotionMagicJerk = HoodConstants.kHoodMotionMagicJerk;

        config.CurrentLimits.StatorCurrentLimit = HoodConstants.kHoodStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = HoodConstants.kHoodStatorCurrentLimitEnable;

        config.CurrentLimits.SupplyCurrentLimit = HoodConstants.kHoodSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = HoodConstants.kHoodSupplyCurrentLowerLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = HoodConstants.kHoodSupplyCurrentLowerTime;
        config.CurrentLimits.SupplyCurrentLimitEnable = HoodConstants.kHoodSupplyCurrentLimitEnable;
        
        hood_motor.getConfigurator().apply(config);
    }

    /** 
     * 讓砲台旋轉到目標角度
     * @param targetRotations 目標轉圈數（例如：轉 1.5 圈，配合齒輪比即為特定角度）
     */
    public Command goToPositionCommand(double targetRotation){
        // 命令馬達走到目標圈數
        return this.run(() -> hood_motor.setControl(m_motionMagicVoltage.withPosition(targetRotation)));
    }
}
