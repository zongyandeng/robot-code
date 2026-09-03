package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
        
        config.Feedback.SensorToMechanismRatio = TurretConstants.kTurretGearRatio; // 設定馬達編碼器與轉塔機構的齒輪比

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // 設定馬達空轉時為煞車模式 (Brake Mode)，防止轉塔自由旋轉
        config.MotorOutput.Inverted = TurretConstants.kTurretMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // 設定馬達旋轉方向是否反向
        // 1. 位置控制 PID 參數設定
        config.Slot0.kP = TurretConstants.kTurretkP; 
        config.Slot0.kI = TurretConstants.kTurretkI; 
        config.Slot0.kD = TurretConstants.kTurretkD; 
        config.Slot0.kV = TurretConstants.kTurretkV;
        config.Slot0.kS = TurretConstants.kTurretkS;

        // 2. 啟用並設定左右軟體限位（正負 90 度對應的圈數，已乘上齒輪比）
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = TurretConstants.kTurretForwardSoftLimitEnable;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.kTurretForwardSoftLimitThreshold;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = TurretConstants.kTurretReverseSoftLimitEnable;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.kTurretReverseSoftLimitThreshold;

        // 3. Motion Magic 運動軌跡控制參數設定（限制最大速度與最大加速度，防機構暴衝）
        config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.kTurretMotionMagicCruiseVelocity; 
        config.MotionMagic.MotionMagicAcceleration = TurretConstants.kTurretMotionMagicAcceleration;
        config.MotionMagic.MotionMagicJerk = TurretConstants.kTurretdMotionMagicJerk; 

        // 4. 定子電流限制設定（Stator Current Limit）：防止物理卡死撞擊時燒毀馬達與損壞齒輪
        config.CurrentLimits.StatorCurrentLimit = TurretConstants.kTurretStatorCurrentLimit; 
        config.CurrentLimits.StatorCurrentLimitEnable = TurretConstants.kTurretStatorCurrentLimitEnable;

        // 5. 電源端電流限制設定（Supply Current Limit）：防止馬達瞬間抽電過大導致電池掉壓 (Brownout)
        config.CurrentLimits.SupplyCurrentLimit = TurretConstants.kTurretSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = TurretConstants.kTurretSupplyCurrentLowerLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = TurretConstants.kTurretSupplyCurrentLowerTime;
        config.CurrentLimits.SupplyCurrentLimitEnable = TurretConstants.kTurretSupplyCurrentLimitEnable;

        turret_motor.getConfigurator().apply(config);

        if (Robot.isSimulation()) {
            m_simState = turret_motor.getSimState();
            // 建立轉塔的物理模型 (二階狀態空間系統：角度與角速度，維度為 N2)
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
            // 1. 讀取馬達驅動電壓輸入給轉臂模擬器
            m_turretSim.setInputVoltage(m_simState.getMotorVoltage());
            m_turretSim.update(0.020);
            
            // 2. 獲取模擬器計算出的物理角度 (Rads) 與角速度 (Rad/Sec)
            double simPosRads = m_turretSim.getAngleRads();
            double simVelRadsPerSec = m_turretSim.getVelocityRadPerSec();
            
            // 3. 轉換單位 (弧度 -> 圈數) 並且乘以齒輪比，得到馬達轉子 (Rotor) 端的數據
            double motorPosRotations = (simPosRads / (2.0 * Math.PI)) * TurretConstants.kTurretGearRatio;
            double motorVelRPS = (simVelRadsPerSec / (2.0 * Math.PI)) * TurretConstants.kTurretGearRatio;
            
            // 4. 將數值同步給馬達的虛擬編碼器
            m_simState.setRawRotorPosition(motorPosRotations);
            m_simState.setRotorVelocity(motorVelRPS);
            
            // 5. 同步虛擬電池電壓
            m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        }
    }    
}