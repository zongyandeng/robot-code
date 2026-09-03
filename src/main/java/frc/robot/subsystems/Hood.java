package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

    private double m_targetPosition = HoodConstants.kHoodStowedPositionRotations; // 目標位置 (圈數)

    private TalonFXSimState m_simState;
    private SingleJointedArmSim m_hoodSim;

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

        if (Robot.isSimulation()) {
            m_simState = hood_motor.getSimState();
            // 建立仰角調整面板的物理模型 (二階狀態空間系統，考慮仰角慣性)
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
    @Override
    public void simulationPeriodic() {
        if (Robot.isSimulation()) {
            // 1. 讀取驅動電壓並更新模擬器物理狀態
            m_hoodSim.setInputVoltage(m_simState.getMotorVoltage());
            m_hoodSim.update(0.020);
            
            // 2. 獲取物理角度 (Rads) 與速度 (Rad/Sec)
            double simPosRads = m_hoodSim.getAngleRads();
            double simVelRadsPerSec = m_hoodSim.getVelocityRadPerSec();
            
            // 3. 轉換為馬達轉子 (Rotor) 端的位置與速度 (乘以齒輪比)
            double motorPosRotations = (simPosRads / (2.0 * Math.PI)) * HoodConstants.kHoodGearRatio;
            double motorVelRPS = (simVelRadsPerSec / (2.0 * Math.PI)) * HoodConstants.kHoodGearRatio;
            
            // 4. 同步至馬達的虛擬編碼器
            m_simState.setRawRotorPosition(motorPosRotations);
            m_simState.setRotorVelocity(motorVelRPS);
            
            // 5. 同步虛擬電池電壓
            m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        }
    }    
}
