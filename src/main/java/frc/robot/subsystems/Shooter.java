package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;;

public class Shooter extends SubsystemBase{
    private final TalonFX topShooter_motor = new TalonFX(ShooterConstants.kTopShooterId);
    private final TalonFX bottomShooter_motor = new TalonFX(ShooterConstants.kBottomShooterId);
    //速度控制物件(用於閉環控制)
    private final VelocityVoltage m_velocityControl = new VelocityVoltage(ShooterConstants.kShooterVelocityVoltage);
    //百分比控制物件(用於開環控制)
    private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(ShooterConstants.kShooterDutyCycleOut);

    public Shooter() {
        var config = new TalonFXConfiguration();
        config.Slot0.kP = ShooterConstants.kShooterkP;  // 比例參數：數值越大，馬達達到目標速度越快
        config.Slot0.kI = ShooterConstants.kShooterkI;   // 積分參數
        config.Slot0.kD = ShooterConstants.kShooterkD; // 微分參數：防震防止轉速衝過頭
        config.Slot0.kV = ShooterConstants.kShooterkV;  // 速度前饋：極為重要！根據馬達的最大轉速與電壓估算 (12V / 最大約 100 RPS)

        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.kShooterStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = ShooterConstants.kShooterStatorCurrentLimitEnable;

        config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.kShooterSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants.kShooterSupplyCurrentLowerLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.kShooterSupplyCurrentLowerTime;
        config.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.kShooterSupplyCurrentLimitEnable;

        //閉環斜率(單位 : 從0V升到12V所需的秒數)
        //例如設為0.25秒，代表馬達電壓不能再0.25秒內邊畫超過12V，這會讓加速更平滑
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ShooterConstants.kShooterVoltageClosedLoopRampPeriod;

        topShooter_motor.getConfigurator().apply(config);
        bottomShooter_motor.getConfigurator().apply(config);
    }

    /**
    * 推薦：使用 PID 閉環速度控制的差速指令
    */
    public Command runShooterVelocityCommand(double baseRPS, double spinFactor){
        return this.run(() -> {
            double topTargetRPS = baseRPS * (1.0 - spinFactor);
            double bottomTargetRPS = baseRPS * (1.0 + spinFactor);

            topShooter_motor.setControl(m_velocityControl.withVelocity(topTargetRPS));
            bottomShooter_motor.setControl(m_velocityControl.withVelocity(bottomTargetRPS));
        });
    }
    
    /**
    * 備用：開環百分比控制的差速指令 (不需調 PID 即可先測試方向與概略輸出)
    */
    public Command runShooterDutyCycleCommand(double baseSpeed, double spinFactor){
        return this.run(() -> {
            double topTargetSpeed = baseSpeed * (1.0 - spinFactor);
            double bottomTargetSpeed = baseSpeed * (1.0 + spinFactor);
            //直接修改屬性欄位 Output
            m_dutyCycleControl.Output = topTargetSpeed;
            topShooter_motor.setControl(m_dutyCycleControl);
            //由於底盤或馬達接收控制請求的機制，建議將上下輪的控制分開送出
            //如果只有一個m_dutyCycleControl，可以連續對output賦值並發送
            m_dutyCycleControl.Output = bottomTargetSpeed;
            bottomShooter_motor.setControl(m_dutyCycleControl);
        });
    }

    //新增的停止命令 (放開按鈕時呼叫)
    public Command stopShooterCommand(){
        return this.runOnce(() -> {
        topShooter_motor.stopMotor();
        bottomShooter_motor.stopMotor();
        });
    }
}