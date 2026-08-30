package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

    private final TalonFX hood_motor = new TalonFX(HoodConstants.kHoodId);

    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(HoodConstants.kHoodMotionMagicVoltage);

    public Hood() {
        var config = new TalonFXConfiguration();
        config.Slot0.kP = HoodConstants.kHoodkP;
        config.Slot0.kI = HoodConstants.kHoodkI;
        config.Slot0.kD = HoodConstants.kHoodkD; 

        config.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.kHoodMotionMagicCruiseVelocity; 
        config.MotionMagic.MotionMagicAcceleration = HoodConstants.kHoodMotionMagicAcceleration; 
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
