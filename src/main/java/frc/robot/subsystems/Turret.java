package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;

public class Turret extends SubsystemBase{
    private final TalonFX turret_motor = new TalonFX(TurretConstants.kTurretId);
    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(TurretConstants.kTurretMotionMagicVoltage);

    //模擬專用變數
    private TalonFXSimState m_simState;
    private SingleJointedArmSim m_turretSim;

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

        if (Robot.isSimulation()) {
            m_simState = turret_motor.getSimState();
            // 建立轉塔的物理模型 (懸臂系統，維度為 N2)
            LinearSystem<N2, N1, N2> turretPlant = LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getKrakenX44(1), 
                TurretConstants.kTurretFlywheelMOI, 
                TurretConstants.kTurretGearRatio
            );
            // 建立單關節懸臂模擬器
            m_turretSim = new SingleJointedArmSim(
                turretPlant,
                DCMotor.getKrakenX44(1),
                TurretConstants.kTurretGearRatio,
                TurretConstants.kTurretArmLengthMeters,
                TurretConstants.kTurretMinAngleRads,
                TurretConstants.kTurretMaxAngleRads,
                false, // 水平旋轉，故設為 false (不考慮重力效應)
                0.0    // 初始角度
            );
        }
    }

    /**
     * 讓砲台選轉到目標角度
     * @param targetRotation 目標轉圈數 (例如 : 轉1.5圈，配合齒輪比即為特定角度)
     */

    public Command goToPositionCommand(double targetRotation) {
        return this.run(() -> turret_motor.setControl(m_motionMagicVoltage.withPosition(targetRotation)));
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Position (Rotations)", turret_motor.getPosition().getValueAsDouble());
    }
    @Override
    public void simulationPeriodic() {
        if (Robot.isSimulation()) {
            m_turretSim.setInputVoltage(m_simState.getMotorVoltage());
            m_turretSim.update(0.020);
            // 轉換弧度到馬達編碼器的圈數 (Rotations = Rads / 2pi)
            // 請注意：馬達端的編碼器位置 = 物理輸出角度 * 齒輪比
            double simPosRads = m_turretSim.getAngleRads();
            double simVelRadsPerSec = m_turretSim.getVelocityRadPerSec();
            double motorPosRotations = (simPosRads / (2.0 * Math.PI)) * 50.0;
            double motorVelRPS = (simVelRadsPerSec / (2.0 * Math.PI)) * 50.0;
            m_simState.setRawRotorPosition(motorPosRotations);
            m_simState.setRotorVelocity(motorVelRPS);
            m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        }
    }    
}