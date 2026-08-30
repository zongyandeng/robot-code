package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase{
    private final TalonFX feeder_motor = new TalonFX(FeederConstants.kFeederId);
    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(FeederConstants.kFeederMotionMagicVoltage);

    // 假設機構與馬達之間的齒輪比為 12:1 (馬達轉 12 圈，分球盤轉 1 圈)
    // 分球盤轉 60 度相當於 1/6 圈，因此馬達需要轉 12 * (1.0 / 6.0) = 2.0 圈
    private final double kRotationsPerStep = FeederConstants.kFeederRotationPerStep;

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
}
