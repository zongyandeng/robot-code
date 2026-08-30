package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;;

public class Turret extends SubsystemBase{
    private final TalonFX turret_motor = new TalonFX(TurretConstants.kTurretId);
    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(TurretConstants.kTurretMotionMagicVoltage);

    public Turret() {
        var config = new TalonFXConfiguration();
        config.Slot0.kP = TurretConstants.kTurretkP; 
        config.Slot0.kI = TurretConstants.kTurretkI; 
        config.Slot0.kD = TurretConstants.kTurretkD; 

        config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.kTurretMotionMagicCruiseVelocity; 
        config.MotionMagic.MotionMagicAcceleration = TurretConstants.kTurretMotionMagicAcceleration;
        config.MotionMagic.MotionMagicJerk = TurretConstants.kTurretdMotionMagicJerk; 

        config.CurrentLimits.StatorCurrentLimit = TurretConstants.kTurretSupplyCurrentLimit; 
        config.CurrentLimits.StatorCurrentLimitEnable = TurretConstants.kTurretStatorCurrentLimitEnable;

        config.CurrentLimits.SupplyCurrentLimit = TurretConstants.kTurretSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = TurretConstants.kTurretSupplyCurrentLowerLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = TurretConstants.kTurretSupplyCurrentLowerTime;
        config.CurrentLimits.SupplyCurrentLimitEnable = TurretConstants.kTurretSupplyCurrentLimitEnable;

        turret_motor.getConfigurator().apply(config);
    }

    /**
     * 讓砲台選轉到目標角度
     * @param targetRotation 目標轉圈數 (例如 : 轉1.5圈，配合齒輪比即為特定角度)
     */

    public Command goToPositionCommand(double targetRotation) {
        return this.run(() -> turret_motor.setControl(m_motionMagicVoltage.withPosition(targetRotation)));
    }
    
}