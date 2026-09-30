package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OrbitConstants;
import frc.robot.Robot;

public class Orbit extends SubsystemBase {
    // 導軌傳送馬達 (Kraken X44)
    private final TalonFX orbit_motor = new TalonFX(OrbitConstants.kOrbitMotorID);

    // 控制請求物件
    private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(0.0);
    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(OrbitConstants.kOrbitMotionMagicVoltage);

    private double m_targetPosition = 0.0;

    // 模擬專用變數
    private TalonFXSimState m_simState;
    private FlywheelSim m_orbitSim;
    private double m_simPosition = 0.0;

    public Orbit() {
        var config = new TalonFXConfiguration();

        // 1. 設定機構齒輪比與空轉煞車模式
        config.Feedback.SensorToMechanismRatio = OrbitConstants.kOrbitGearRatio;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = OrbitConstants.kOrbitMotorInverted 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;

        // 2. 位置控制與前饋 PID 參數 (Slot 0)
        config.Slot0.kP = OrbitConstants.kOrbitkP;
        config.Slot0.kI = OrbitConstants.kOrbitkI;
        config.Slot0.kD = OrbitConstants.kOrbitkD;
        config.Slot0.kV = OrbitConstants.kOrbitkV;
        config.Slot0.kS = OrbitConstants.kOrbitkS;

        // 3. Motion Magic 運動曲線參數
        config.MotionMagic.MotionMagicCruiseVelocity = OrbitConstants.kOrbitMotionMagicCruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = OrbitConstants.kOrbitMotionMagicAcceleration;
        config.MotionMagic.MotionMagicJerk = OrbitConstants.kOrbitMotionMagicJerk;

        // 4. 定子電流限制 (保護馬達與機構防卡球燒毀)
        config.CurrentLimits.StatorCurrentLimit = OrbitConstants.kOrbitStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = OrbitConstants.kOrbitStatorCurrentLimitEnable;

        // 5. 電源端電流限制 (防止大電流瞬間抽乾電池電壓)
        config.CurrentLimits.SupplyCurrentLimit = OrbitConstants.kOrbitSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = OrbitConstants.kOrbitSupplyCurrentLowerLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = OrbitConstants.kOrbitSupplyCurrentLowerTime;
        config.CurrentLimits.SupplyCurrentLimitEnable = OrbitConstants.kOrbitSupplyCurrentLimitEnable;

        orbit_motor.getConfigurator().apply(config);

        // 僅在桌面模擬模式下初始化物理模擬器
        if (Robot.isSimulation()) {
            m_simState = orbit_motor.getSimState();
            // 建立導軌滾輪系統物理模型 (狀態空間表示法，Kraken X44 單馬達)
            LinearSystem<N1, N1, N1> orbitPlant = LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX44(1),
                OrbitConstants.kOrbitFlywheelMOI,
                OrbitConstants.kOrbitGearRatio
            );
            m_orbitSim = new FlywheelSim(orbitPlant, DCMotor.getKrakenX44(1));
        }
    }

    /**
     * 以指定功率比率運轉導軌 (開環 DutyCycle)
     * @param speed 輸出功率 (-1.0 ~ 1.0)
     */
    public Command runOrbitCommand(double speed) {
        return this.runEnd(
            () -> orbit_motor.setControl(m_dutyCycleControl.withOutput(speed)),
            () -> orbit_motor.stopMotor()
        );
    }

    /**
     * 以預設功率送球推進至 Shooter
     */
    public Command feedCommand() {
        return runOrbitCommand(OrbitConstants.kOrbitFeedDutyCycle);
    }

    /**
     * 卡球反轉排障命令
     */
    public Command reverseCommand() {
        return runOrbitCommand(OrbitConstants.kOrbitReverseDutyCycle);
    }

    /**
     * 停止導軌馬達
     */
    public Command stopOrbitCommand() {
        return this.runOnce(orbit_motor::stopMotor);
    }

    /**
     * 讓導軌轉動到指定圈數 (Motion Magic 位置控制)
     * @param targetRotations 目標轉圈數
     */
    public Command goToPositionCommand(double targetRotations) {
        return this.run(() -> {
            m_targetPosition = targetRotations;
            orbit_motor.setControl(m_motionMagicVoltage.withPosition(m_targetPosition));
        });
    }

    /**
     * 判斷是否已達到目標位置
     */
    public boolean isAtTargetPosition() {
        double currentPosition = orbit_motor.getPosition().getValueAsDouble();
        return Math.abs(currentPosition - m_targetPosition) < OrbitConstants.kOrbitPositionToleranceRotations;
    }

    @Override
    public void periodic() {
        // 發布遙測數據到 SmartDashboard
        SmartDashboard.putNumber("Orbit/Velocity (RPS)", orbit_motor.getRotorVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Orbit/Position (Rotations)", orbit_motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Orbit/Current (A)", orbit_motor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        if (Robot.isSimulation()) {
            // 1. 讀取馬達驅動電壓傳入物理模擬器
            m_orbitSim.setInputVoltage(m_simState.getMotorVoltage());
            m_orbitSim.update(0.020);

            // 2. 獲取導軌軸的模擬轉速 (RPS)
            double simRPS = m_orbitSim.getAngularVelocityRPM() / 60.0;

            // 3. 乘上齒輪比得到轉子端轉速
            double motorVelRPS = simRPS * OrbitConstants.kOrbitGearRatio;
            m_simState.setRotorVelocity(motorVelRPS);

            // 4. 速度積分累積為虛擬位置
            m_simPosition += motorVelRPS * 0.020;
            m_simState.setRawRotorPosition(m_simPosition);

            // 5. 同步虛擬電池電壓
            m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        }
    }
}
