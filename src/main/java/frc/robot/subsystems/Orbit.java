package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OrbitConstants;

public class Orbit extends SubsystemBase {
    // 導軌傳送馬達 (Kraken X44)
    private final TalonFX orbit_motor = new TalonFX(OrbitConstants.kOrbitMotorID);
    // 第二顆導軌馬達（跟隨主馬達）
    private final TalonFX orbit_follower = new TalonFX(OrbitConstants.kOrbitFollowerMotorID);

    // 控制請求物件
    private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(0.0);
    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(OrbitConstants.kOrbitMotionMagicVoltage);

    private double m_targetPosition = 0.0;

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
        orbit_follower.getConfigurator().apply(config);

    }

    // 跟隨請求：僅位置控制時使用（第二顆馬達跟著主馬達轉）
    private final Follower m_followerRequest = new Follower(
        OrbitConstants.kOrbitMotorID,
        OrbitConstants.kOrbitFollowerOpposed ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned);

    // 第二顆馬達的開環請求（直接給輸出，不靠跟隨模式）
    private final DutyCycleOut m_followerDutyCycle = new DutyCycleOut(0.0);

    /**
     * 以指定功率比率運轉導軌 (開環 DutyCycle)：兩顆馬達各自直接輸出，第二顆依設定可反向
     * @param speed 輸出功率 (-1.0 ~ 1.0)
     */
    public Command runOrbitCommand(double speed) {
        return this.runEnd(
            () -> {
                orbit_motor.setControl(m_dutyCycleControl.withOutput(speed));
                orbit_follower.setControl(m_followerDutyCycle.withOutput(
                    OrbitConstants.kOrbitFollowerOpposed ? -speed : speed));
            },
            this::stopAll
        );
    }

    private void stopAll() {
        orbit_motor.stopMotor();
        orbit_follower.stopMotor();
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
        return this.runOnce(this::stopAll);
    }

    /**
     * 讓導軌轉動到指定圈數 (Motion Magic 位置控制)
     * @param targetRotations 目標轉圈數
     */
    public Command goToPositionCommand(double targetRotations) {
        return this.run(() -> {
            m_targetPosition = targetRotations;
            orbit_motor.setControl(m_motionMagicVoltage.withPosition(m_targetPosition));
            orbit_follower.setControl(m_followerRequest);
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
        // 第二顆馬達診斷數據：判斷它有沒有收到指令、有沒有出力、有沒有故障
        SmartDashboard.putNumber("Orbit2/Current (A)", orbit_follower.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Orbit2/Motor Voltage (V)", orbit_follower.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Orbit2/Velocity (RPS)", orbit_follower.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Orbit2/Supply Voltage (V)", orbit_follower.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Orbit2/Fault Field", orbit_follower.getFaultField().getValueAsDouble());
        SmartDashboard.putNumber("Orbit2/Sticky Fault Field", orbit_follower.getStickyFaultField().getValueAsDouble());
        SmartDashboard.putBoolean("Orbit2/Connected", orbit_follower.isConnected());

        // 發布遙測數據到 SmartDashboard
        SmartDashboard.putNumber("Orbit/Velocity (RPS)", orbit_motor.getRotorVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Orbit/Position (Rotations)", orbit_motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Orbit/Current (A)", orbit_motor.getStatorCurrent().getValueAsDouble());
    }

}
