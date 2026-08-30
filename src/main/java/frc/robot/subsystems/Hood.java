package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.RobotController;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

    private final TalonFX hood_motor = new TalonFX(HoodConstants.kHoodId);

    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(HoodConstants.kHoodMotionMagicVoltage);

    private TalonFXSimState m_simState;
    private SingleJointedArmSim m_hoodSim;

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

        if (Robot.isSimulation()) {
            m_simState = hood_motor.getSimState();
            // 建立仰角面板的物理模型
            LinearSystem<N2, N1, N2> hoodPlant = LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getKrakenX44(1), 
                HoodConstants.kHoodFlywheelMOI, 
                HoodConstants.kHoodGearRatio
            );
            // 建立單關節懸臂模擬器
            m_hoodSim = new SingleJointedArmSim(
                hoodPlant,
                DCMotor.getKrakenX44(1),
                HoodConstants.kHoodGearRatio,
                HoodConstants.kHoodArmLengthMeters,
                HoodConstants.kHoodMinAngleRads,
                HoodConstants.kHoodMaxAngleRads,
                true, // 砲面板垂直擺動會受重力引力下墜，故設為 true (考慮重力效應)
                0.0   // 初始角度
            );
        }   
    }

    /** 
     * 讓砲台旋轉到目標角度
     * @param targetRotations 目標轉圈數（例如：轉 1.5 圈，配合齒輪比即為特定角度）
     */
    public Command goToPositionCommand(double targetRotation){
        // 命令馬達走到目標圈數
        return this.run(() -> hood_motor.setControl(m_motionMagicVoltage.withPosition(targetRotation)));
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood/Position (Rotations)", hood_motor.getPosition().getValueAsDouble());
    }
    @Override
    public void simulationPeriodic() {
        if (Robot.isSimulation()) {
            m_hoodSim.setInputVoltage(m_simState.getMotorVoltage());
            m_hoodSim.update(0.020);
            double simPosRads = m_hoodSim.getAngleRads();
            double simVelRadsPerSec = m_hoodSim.getVelocityRadPerSec();
            // 馬達位置與速度 = 物理輸出角度/速度 * 齒輪比 (80.0)
            double motorPosRotations = (simPosRads / (2.0 * Math.PI)) * HoodConstants.kHoodGearRatio;
            double motorVelRPS = (simVelRadsPerSec / (2.0 * Math.PI)) * HoodConstants.kHoodGearRatio;
            m_simState.setRawRotorPosition(motorPosRotations);
            m_simState.setRotorVelocity(motorVelRPS);
            m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        }
    }    
}
