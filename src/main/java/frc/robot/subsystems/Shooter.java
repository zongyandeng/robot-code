package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    private final TalonFX topShooter_motor = new TalonFX(ShooterConstants.kTopShooterId);
    private final TalonFX bottomShooter_motor = new TalonFX(ShooterConstants.kBottomShooterId);

    private final MotionMagicVelocityVoltage m_velocityControl = new MotionMagicVelocityVoltage(ShooterConstants.kShooterVelocityVoltage);
    private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(ShooterConstants.kShooterDutyCycleOut);

    //用於發佈到圖表的目標轉速追蹤變數
    private double m_topTargetRPS = ShooterConstants.kTopShooterTargetRPS;
    private double m_bottomTargetRPS = ShooterConstants.kBottomShooterTargetRPS;

    public Shooter() {
        var topConfig = new TalonFXConfiguration();
        topConfig.Slot0.kP = ShooterConstants.kShooterkP;  
        topConfig.Slot0.kI = ShooterConstants.kShooterkI; 
        topConfig.Slot0.kD = ShooterConstants.kShooterkD; 
        topConfig.Slot0.kV = ShooterConstants.kShooterkV;  
        topConfig.Slot0.kS = ShooterConstants.kShooterkS;  

        topConfig.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kShooterCruiseVelocity;
        topConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.kShooterAcceleration;
        topConfig.MotionMagic.MotionMagicJerk = ShooterConstants.kShooterJerk;

        topConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.kShooterStatorCurrentLimit;
        topConfig.CurrentLimits.StatorCurrentLimitEnable = ShooterConstants.kShooterStatorCurrentLimitEnable;

        topConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.kShooterSupplyCurrentLimit;
        topConfig.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants.kShooterSupplyCurrentLowerLimit;
        topConfig.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.kShooterSupplyCurrentLowerTime;
        topConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.kShooterSupplyCurrentLimitEnable;

        topConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ShooterConstants.kShooterVoltageClosedLoopRampPeriod;

        // 設定上發射輪馬達轉向並套用配置
        topConfig.MotorOutput.Inverted = ShooterConstants.kTopShooterInverted 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        topShooter_motor.getConfigurator().apply(topConfig);

        // 下發射輪複製相同控制參數，套用反向設定 (面對面夾球發射)
        var bottomConfig = new TalonFXConfiguration();
        bottomConfig.Slot0 = topConfig.Slot0;
        bottomConfig.MotionMagic = topConfig.MotionMagic;
        bottomConfig.CurrentLimits = topConfig.CurrentLimits;
        bottomConfig.ClosedLoopRamps = topConfig.ClosedLoopRamps;
        bottomConfig.MotorOutput.Inverted = ShooterConstants.kBottomShooterInverted 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        bottomShooter_motor.getConfigurator().apply(bottomConfig);

    }

    public double getTopRPS(){
        return topShooter_motor.getRotorVelocity().getValueAsDouble();
    }

    public double getBottomRPS(){
        return bottomShooter_motor.getRotorVelocity().getValueAsDouble();
    }

    public boolean isAtTargetVelocity() {
        if(m_topTargetRPS == 0.0 && m_bottomTargetRPS == 0.0){
            return false; //當目標轉速為 0 時，視為未達到目標速度，避免誤判
        }
        boolean topReady = Math.abs(getTopRPS() - m_topTargetRPS) <= ShooterConstants.kShooterVelocityToleranceRPS;
        boolean bottomReady = Math.abs(getBottomRPS() - m_bottomTargetRPS) <= ShooterConstants.kShooterVelocityToleranceRPS;
        return topReady && bottomReady;
    }

    /**
    * 推薦：使用 PID 閉環速度控制的差速指令
    */
    /** 同 runShooterVelocityCommand，但基準轉速每個週期重讀（填表時從儀表板即時調整轉速用）。 */
    public Command runShooterVelocityCommand(DoubleSupplier baseRPS, double spinFactor){
        return this.runEnd(() -> {
            double rps = baseRPS.getAsDouble();
            m_topTargetRPS = rps * (1.0 - spinFactor);
            m_bottomTargetRPS = rps * (1.0 + spinFactor);
            topShooter_motor.setControl(m_velocityControl.withVelocity(m_topTargetRPS));
            bottomShooter_motor.setControl(m_velocityControl.withVelocity(m_bottomTargetRPS));
        },
        () -> {
            m_topTargetRPS = 0.0;
            m_bottomTargetRPS = 0.0;
            topShooter_motor.stopMotor();
            bottomShooter_motor.stopMotor();
        });
    }

    public Command runShooterVelocityCommand(double baseRPS, double spinFactor){
        return this.runEnd(() -> {
            m_topTargetRPS = baseRPS * (1.0 - spinFactor);
            m_bottomTargetRPS = baseRPS * (1.0 + spinFactor);

            topShooter_motor.setControl(m_velocityControl.withVelocity(m_topTargetRPS));
            bottomShooter_motor.setControl(m_velocityControl.withVelocity(m_bottomTargetRPS));
        },
        () -> {
            //當命令結束時，將目標轉速重置為 0，並停止馬達
            m_topTargetRPS = 0.0;
            m_bottomTargetRPS = 0.0;
            topShooter_motor.stopMotor();
            bottomShooter_motor.stopMotor();
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
        m_topTargetRPS = 0.0;
        m_bottomTargetRPS = 0.0;
        topShooter_motor.stopMotor();
        bottomShooter_motor.stopMotor();
        });
    }

    @Override
    public void periodic() {
        //發布數據到 SmartDashboard，以便Glass / AdvantageScope繪製折線圖
        SmartDashboard.putNumber("Shooter/TopTargetRPS", m_topTargetRPS);
        SmartDashboard.putNumber("Shooter/Top Actual RPS", topShooter_motor.getRotorVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/BottomTargetRPS", m_bottomTargetRPS);
        SmartDashboard.putNumber("Shooter/Bottom Actual RPS", bottomShooter_motor.getRotorVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Shooter/At Target Velocity", isAtTargetVelocity());
    }

}