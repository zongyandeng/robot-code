package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.RobotController;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase{
    private final TalonFX feeder_motor = new TalonFX(FeederConstants.kFeederId);
    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(FeederConstants.kFeederMotionMagicVoltage);

    private final double kRotationsPerStep = FeederConstants.kFeederRotationPerStep;

    //模擬專用變數
    private TalonFXSimState m_simState;
    private FlywheelSim m_FlywheelSim;

    public Feeder() {
        var config = new TalonFXConfiguration();
        config.Slot0.kP = FeederConstants.kFeederkP;
        config.Slot0.kI = FeederConstants.kFeederkI;
        config.Slot0.kD = FeederConstants.kFeederkD;

        config.MotionMagic.MotionMagicCruiseVelocity = FeederConstants.kFeederMotionMagicCruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = FeederConstants.kFeederMotionMagicAcceleration;
        config.MotionMagic.MotionMagicJerk = FeederConstants.kFeederMotionMagicJerk;

        config.CurrentLimits.StatorCurrentLimit = FeederConstants.kFeederStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = FeederConstants.kFeederStatorCurrentLimitEnable;

        config.CurrentLimits.SupplyCurrentLimit = FeederConstants.kFeederSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = FeederConstants.kFeederSupplyCurrentLowerLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = FeederConstants.kFeederSupplyCurrentLowerTime;
        config.CurrentLimits.SupplyCurrentLimitEnable = FeederConstants.kFeederSupplyCurrentLimitEnable;

        feeder_motor.getConfigurator().apply(config);

        if (Robot.isSimulation()) {
            m_simState = feeder_motor.getSimState();
            // 建立 Feeder 的物理模型並初始化模擬器
            LinearSystem<N1, N1, N1> feederPlant = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44(1), FeederConstants.kFeederFlywheelMOI, FeederConstants.kFeederGearRatio);
            m_FlywheelSim = new FlywheelSim(feederPlant, DCMotor.getKrakenX44(1));
}
    }

    /** 
     * 步進指令：每次呼叫，馬達就精確往前多轉 60 度（也就是馬達轉動 kRotationsPerStep 圈）
     */
    public Command stepForCommand() {
        return this.defer(() -> {  //this.defer是WPIlib很方便的語法，他能確保在按鍵按下的瞬間才去讀取currentPos，而不是程式開機時就決定好
            //1.讀取當前瞬間馬達的實際位置(圈數)
            double currentPos = feeder_motor.getPosition().getValueAsDouble();
            //2.計算目標位置 = 當前位置 + 增量
            double targetPos = currentPos + kRotationsPerStep;
            //3.執行指令 : 走到目標位置
            return this.run(() -> feeder_motor.setControl(m_motionMagicVoltage.withPosition(targetPos)))
                //當馬達非常接近目標時(誤差小於0.05圈)，就判定此動作已完成並結束指令
                .until(() -> Math.abs(feeder_motor.getPosition().getValueAsDouble() - targetPos) < 0.05);
        });
    }

    @Override
    public void periodic() {
        // 發布位置，讓我們能檢查當步進按鈕按下時，位置是否每次增加 2 圈後精確停下
        SmartDashboard.putNumber("Feeder/Position (Rotations)", feeder_motor.getPosition().getValueAsDouble());
    }
    @Override
    public void simulationPeriodic() {
        if (Robot.isSimulation()) {
            m_FlywheelSim.setInputVoltage(m_simState.getMotorVoltage());
            m_FlywheelSim.update(0.020);
            // 速度 (圈/秒)
            double simRPS = m_FlywheelSim.getAngularVelocityRPM() / 60.0;
            m_simState.setRotorVelocity(simRPS);
            // 位置 (圈)：當前編碼器位置 + (速度 * 時間差)
            double currentPos = feeder_motor.getPosition().getValueAsDouble();
            m_simState.setRawRotorPosition(currentPos + simRPS * 0.020);
            m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        }
    }
}
