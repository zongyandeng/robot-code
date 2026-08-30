package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;;

public class Turret extends SubsystemBase{
    private final TalonFX turret_motor = new TalonFX(TurretConstants.kTurretId);
    //建立一個位置控制請求物件
    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(TurretConstants.kTurretMotionMagicVoltage);

    public Turret() {
        //設定馬達的PID參數，讓馬達知道如何自我微調、不衝過頭
        var config = new TalonFXConfiguration();
        config.Slot0.kP = TurretConstants.kTurretkP; //比例參數:數值越大，馬達反應越快，但太大會抖動
        config.Slot0.kI = TurretConstants.kTurretkI; //積分參數
        config.Slot0.kD = TurretConstants.kTurretkD; //微分參數:用來減速防震，防止馬達衝過頭

        //設定Motion Magic 的速度與加速度上限
        //數值單位為馬達圈數/秒(Rotations per second)，須根據實際機構齒輪比與安全需求調整
        config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.kTurretMotionMagicCruiseVelocity; //最大巡航速度(每秒最多轉10圈)
        config.MotionMagic.MotionMagicAcceleration = TurretConstants.kTurretMotionMagicAcceleration; //最大加速度 (0.5秒內可以加速到的最大速度)
        config.MotionMagic.MotionMagicJerk = TurretConstants.kTurretdMotionMagicJerk; //急動度(Jerk，設為0代表不限制)

        //設訂定子電流限制，防止瞬間電流過大造成 Brownout 或燒馬達
        config.CurrentLimits.StatorCurrentLimit = TurretConstants.kTurretSupplyCurrentLimit; //限制電流最大40A
        config.CurrentLimits.StatorCurrentLimitEnable = TurretConstants.kTurretStatorCurrentLimitEnable;

        config.CurrentLimits.SupplyCurrentLimit = TurretConstants.kTurretSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = TurretConstants.kTurretSupplyCurrentLowerLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = TurretConstants.kTurretSupplyCurrentLowerTime;
        config.CurrentLimits.SupplyCurrentLimitEnable = TurretConstants.kTurretSupplyCurrentLimitEnable;

        turret_motor.getConfigurator().apply(config);
    }

    /**
     * 讓砲台選轉到目標角度
     * @param targetRotation 目標轉圈數 (例如 : 轉1.5圈，配合齒輪比即為特定角度)
     */

    public Command goToPositionCommand(double targetRotation) {
        return this.run(() -> turret_motor.setControl(m_motionMagicVoltage.withPosition(targetRotation)));
    }
    
}