package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase{
    private final TalonFX feeder_motor = new TalonFX(FeederConstants.kFeederId);
    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(FeederConstants.kFeederMotionMagicVoltage);

    private final double kRotationsPerStep = FeederConstants.kFeederRotationPerStep;

    public Feeder() {
        var config = new TalonFXConfiguration();

        config.Feedback.SensorToMechanismRatio = FeederConstants.kFeederGearRatio; // 設定馬達編碼器與分球盤的齒輪比
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // 設定馬達空轉時為煞車模式 (Brake Mode)，防止分球盤自由旋轉

        config.Slot0.kP = FeederConstants.kFeederkP;
        config.Slot0.kI = FeederConstants.kFeederkI;
        config.Slot0.kD = FeederConstants.kFeederkD;
        config.Slot0.kV = FeederConstants.kFeederkV;
        config.Slot0.kS = FeederConstants.kFeederkS;

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

    }

    /** 
     * 步進指令：每次呼叫，馬達就精確往前多轉 60 度（也就是馬達轉動 kRotationsPerStep 圈）
     */
    public Command stepForCommand() {
        return this.defer(() -> {  //this.defer是WPIlib很方便的語法，他能確保在按鍵按下的瞬間才去讀取currentPos，而不是程式開機時就決定好
            //1.讀取當前瞬間馬達的實際位置(圈數)
            double currentPos = feeder_motor.getPosition().getValueAsDouble();
            //2.計算目標位置 = 當前位置 + 增量
            double targetPos = (Math.round(currentPos / kRotationsPerStep) + 1.0) * kRotationsPerStep; // 先將當前位置除以步進圈數，四捨五入取整數，再加 1，最後乘回步進圈數，得到下一個目標位置
            //3.執行指令 : 走到目標位置 (加入 1.0 秒安全逾時，避免微小誤差或卡球導致指令永不結束)
            return this.run(() -> feeder_motor.setControl(m_motionMagicVoltage.withPosition(targetPos)))
                //當馬達非常接近目標時(誤差小於容忍度)，就判定此動作已完成並結束指令
                .until(() -> Math.abs(feeder_motor.getPosition().getValueAsDouble() - targetPos) < FeederConstants.kFeederPositionToleranceRotations)
                .withTimeout(1.0);
        });
    }

    public Command reverseCommand() {
        return this.runEnd(
            () -> feeder_motor.set(-0.3),
            () -> feeder_motor.stopMotor());
    }

    public Command stopFeederCommand() {
        return this.runOnce(feeder_motor::stopMotor);
    }

    @Override
    public void periodic() {
        // 發布位置，讓我們能檢查當步進按鈕按下時，位置是否每次增加 2 圈後精確停下
        SmartDashboard.putNumber("Feeder/Position (Rotations)", feeder_motor.getPosition().getValueAsDouble());
    }
}
