package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase{
    private final TalonFX feeder_motor = new TalonFX(FeederConstants.kFeederId);
    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(FeederConstants.kFeederMotionMagicVoltage);

    private final double kRotationsPerStep = FeederConstants.kFeederRotationPerStep;

    private int m_jamCount = 0; // 卡球次數（儀表板顯示，方便調整偵測靈敏度）
    private String m_state = "待機"; // 目前步進狀態（儀表板顯示，方便除錯）
    private double m_homePos = 0.0;  // 預設位置：最近一次步進開始前（或順利完成後）停的格線位置，卡球時退回這裡

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
        feeder_motor.setPosition(0.0); // 開電初始化：把分球盤當下位置當作 0 圈

        // 在 SmartDashboard 提供一鍵歸零按鈕 (Disabled 狀態下也能按)
        SmartDashboard.putData("Feeder/Zero Position", zeroPositionCommand().ignoringDisable(true));

        // 「馬達叫」監視：沒有任何指令在跑、電流大、卻幾乎不轉（球頂著分球盤，馬達硬撐），
        // 持續一小段時間就自動反轉解卡並退回預設位置。步進途中的卡球由步進指令自己處理。
        new Trigger(() -> getCurrentCommand() == null && isWhining())
            .debounce(FeederConstants.kFeederWhineTimeSec)
            .onTrue(idleUnjamCommand());
    }

    /** 馬達在「叫」：電流大於門檻、但幾乎不轉 */
    private boolean isWhining() {
        return feeder_motor.getStatorCurrent().getValueAsDouble() > FeederConstants.kFeederWhineCurrentAmps
            && Math.abs(feeder_motor.getVelocity().getValueAsDouble()) < FeederConstants.kFeederStallVelocityRPS;
    }

    /** 待機中偵測到「叫」：開環反轉解卡，再退回預設位置，最後放開馬達停止硬撐 */
    private Command idleUnjamCommand() {
        return this.runOnce(() -> {
                m_jamCount++;
                m_state = "待機中偵測到叫，反轉解卡";
            })
            .andThen(this.runEnd(
                    () -> feeder_motor.set(-FeederConstants.kFeederJamReverseDuty),
                    () -> feeder_motor.stopMotor())
                .until(() -> feeder_motor.getPosition().getValueAsDouble() <= m_homePos) // 退到預設位置就停，不多轉
                .withTimeout(FeederConstants.kFeederJamReverseSec))
            .andThen(Commands.defer(() -> moveTo(m_homePos, FeederConstants.kFeederRecoverTimeoutSec), java.util.Set.of(this)))
            .andThen(this.runOnce(() -> {
                feeder_motor.stopMotor(); // 不再硬撐目標位置，避免一直叫（煞車模式仍會擋住分球盤）
                m_state = "待機";
            }));
    }

    /** 手動將分球盤當前位置重新校準歸零為 0 圈 */
    public Command zeroPositionCommand() {
        return this.runOnce(() -> {
            feeder_motor.setPosition(0.0);
            m_homePos = 0.0;
            feeder_motor.setControl(m_motionMagicVoltage.withPosition(0.0)); // 目標也回到 0，避免啟用時衝回舊目標
        });
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
            // 這一步開始前的預設位置（= 目標往回一格），卡球時要退回到這裡
            double startPos = targetPos - kRotationsPerStep;
            m_homePos = startPos;
            //3.執行步進；卡球時自動退回預設位置再重試
            return stepWithJamRecovery(startPos, targetPos, FeederConstants.kFeederJamMaxRetries);
        });
    }

    private boolean isNear(double pos) {
        return Math.abs(feeder_motor.getPosition().getValueAsDouble() - pos) < FeederConstants.kFeederPositionToleranceRotations;
    }

    /** 走到指定位置（有逾時保護） */
    private Command moveTo(double pos, double timeoutSec) {
        return this.run(() -> feeder_motor.setControl(m_motionMagicVoltage.withPosition(pos)))
            .until(() -> isNear(pos))
            .withTimeout(timeoutSec);
    }

    /**
     * 帶卡球偵測的步進：
     *  - 走向目標位置時，若電流持續過大（堵轉）或逾時沒到位，就判定「卡球」
     *  - 卡球時先退回這一步開始前的預設位置，再重新嘗試，最多重試 retriesLeft 次
     *  - 重試用完仍卡住，就退回預設位置並放棄（不再硬推，保護馬達與機構）
     */
    private Command stepWithJamRecovery(double startPos, double targetPos, int retriesLeft) {
        final boolean[] jammed = {false};
        final Debouncer jamDebouncer = new Debouncer(FeederConstants.kFeederJamTimeSec, Debouncer.DebounceType.kRising);
        final Debouncer stallDebouncer = new Debouncer(FeederConstants.kFeederStallTimeSec, Debouncer.DebounceType.kRising);

        Command attempt = this.run(() -> feeder_motor.setControl(m_motionMagicVoltage.withPosition(targetPos)))
            .until(() -> {
                // 偵測 1：電流持續過大（堵轉）
                boolean overCurrent = feeder_motor.getStatorCurrent().getValueAsDouble() > FeederConstants.kFeederJamCurrentAmps;
                // 偵測 2：還沒到位，但幾乎不動（卡住不轉，不管電流多少）
                boolean stalled = !isNear(targetPos)
                    && Math.abs(feeder_motor.getVelocity().getValueAsDouble()) < FeederConstants.kFeederStallVelocityRPS;
                if (jamDebouncer.calculate(overCurrent) | stallDebouncer.calculate(stalled)) {
                    jammed[0] = true;
                }
                return jammed[0] || isNear(targetPos);
            })
            .withTimeout(FeederConstants.kFeederStepTimeoutSec);

        // 卡球時：先「開環」直接用固定功率反轉一小段時間（不看位置），讓球鬆開。
        // 不用位置控制退回：卡住時位置控制的力道不夠、又可能根本沒離開起點，會變成原地硬推。
        Command recover = Commands.runOnce(() -> {
                m_jamCount++;
                m_state = "卡球，反轉解卡";
            })
            .andThen(this.runEnd(
                    () -> feeder_motor.set(-FeederConstants.kFeederJamReverseDuty),
                    () -> feeder_motor.stopMotor())
                .until(() -> feeder_motor.getPosition().getValueAsDouble() <= startPos) // 退到預設位置就停，不多轉
                .withTimeout(FeederConstants.kFeederJamReverseSec));
        if (retriesLeft > 0) {
            // 還有重試次數：解卡後直接再往目標推一次
            recover = recover.andThen(stepWithJamRecovery(startPos, targetPos, retriesLeft - 1));
        } else {
            // 重試用完：放棄，回到預設位置
            recover = recover
                .andThen(Commands.runOnce(() -> m_state = "放棄，回預設位置"))
                .andThen(moveTo(startPos, FeederConstants.kFeederRecoverTimeoutSec));
        }

        return Commands.runOnce(() -> m_state = "前進")
            .andThen(attempt)
            .andThen(Commands.either(
                Commands.runOnce(() -> {                      // 順利到位：結束，新的預設位置 = 目標位置
                    m_state = "完成";
                    m_homePos = targetPos;
                }),
                recover,                                     // 卡住或沒到位：反轉解卡（並重試）
                () -> !jammed[0] && isNear(targetPos)));
    }

    /**
     * 反向步進指令：每次呼叫，馬達精確往回轉一格（kRotationsPerStep 圈，預設 120 度）
     */
    public Command stepBackCommand() {
        return this.defer(() -> {
            double currentPos = feeder_motor.getPosition().getValueAsDouble();
            double targetPos = (Math.round(currentPos / kRotationsPerStep) - 1.0) * kRotationsPerStep;
            return this.run(() -> feeder_motor.setControl(m_motionMagicVoltage.withPosition(targetPos)))
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
        SmartDashboard.putNumber("Feeder/Current (A)", feeder_motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/Jam Count", m_jamCount);
        SmartDashboard.putString("Feeder/State", m_state);
        SmartDashboard.putNumber("Feeder/Velocity (RPS)", feeder_motor.getVelocity().getValueAsDouble());
    }
}
