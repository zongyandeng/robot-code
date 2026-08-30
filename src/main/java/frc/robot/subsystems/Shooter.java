package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.LinearSystem;


import frc.robot.Constants.ShooterConstants;;

public class Shooter extends SubsystemBase{
    private final TalonFX topShooter_motor = new TalonFX(ShooterConstants.kTopShooterId);
    private final TalonFX bottomShooter_motor = new TalonFX(ShooterConstants.kBottomShooterId);

    private final VelocityVoltage m_velocityControl = new VelocityVoltage(ShooterConstants.kShooterVelocityVoltage);
    private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(ShooterConstants.kShooterDutyCycleOut);

    //用於發佈到圖表的目標轉速追蹤變數
    private double m_topTargetRPS = ShooterConstants.kTopShooterTargetRPS;
    private double m_bottomTargetRPS = ShooterConstants.kBottomShooterTargetRPS;

    //模擬專用變數
    private TalonFXSimState m_topSimState;
    private TalonFXSimState m_bottomSimState;
    private FlywheelSim m_topFlywheelSim;
    private FlywheelSim m_bottomFlywheelSim;

    public Shooter() {
        var config = new TalonFXConfiguration();
        config.Slot0.kP = ShooterConstants.kShooterkP;  
        config.Slot0.kI = ShooterConstants.kShooterkI; 
        config.Slot0.kD = ShooterConstants.kShooterkD; 
        config.Slot0.kV = ShooterConstants.kShooterkV;  

        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.kShooterStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = ShooterConstants.kShooterStatorCurrentLimitEnable;

        config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.kShooterSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants.kShooterSupplyCurrentLowerLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.kShooterSupplyCurrentLowerTime;
        config.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.kShooterSupplyCurrentLimitEnable;

        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ShooterConstants.kShooterVoltageClosedLoopRampPeriod;

        topShooter_motor.getConfigurator().apply(config);
        bottomShooter_motor.getConfigurator().apply(config);

        //僅在模擬環境下初始化物理模擬器
        if(Robot.isSimulation()){
            m_topSimState = topShooter_motor.getSimState();
            m_bottomSimState = bottomShooter_motor.getSimState();

            //建立上輪物理模型(plant)並初始化模擬器(馬達數量設為1)
            LinearSystem<N1, N1, N1> topFlywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44(1), ShooterConstants.kTopShooterFlywheelMOI, ShooterConstants.kTopShooterGearRatio);
            m_topFlywheelSim = new FlywheelSim(topFlywheelPlant, DCMotor.getKrakenX44(1));
            //建立下輪物理模型(plant)並初始化模擬器(馬達數量設為1)
            LinearSystem<N1, N1, N1> bottomFlywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44(1), ShooterConstants.kBottomShooterFlywheelMOI, ShooterConstants.kBottomShooterGearRatio);
            m_bottomFlywheelSim = new FlywheelSim(bottomFlywheelPlant, DCMotor.getKrakenX44(1));
        }
    }

    /**
    * 推薦：使用 PID 閉環速度控制的差速指令
    */
    public Command runShooterVelocityCommand(double baseRPS, double spinFactor){
        return this.run(() -> {
            m_topTargetRPS = baseRPS * (1.0 - spinFactor);
            m_bottomTargetRPS = baseRPS * (1.0 + spinFactor);

            topShooter_motor.setControl(m_velocityControl.withVelocity(m_topTargetRPS));
            bottomShooter_motor.setControl(m_velocityControl.withVelocity(m_bottomTargetRPS));
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

    @Override
    public void periodic() {
        //發布數據到 SmartDashboard，以便Glass / AdvantageScope繪製折線圖
        SmartDashboard.putNumber("Shooter/TopTargetRPS", m_topTargetRPS);
        SmartDashboard.putNumber("Shooter/Top Actual RPS", topShooter_motor.getRotorVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/BottomTargetRPS", m_bottomTargetRPS);
        SmartDashboard.putNumber("Shooter/Bottom Actual RPS", bottomShooter_motor.getRotorVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        if(Robot.isSimulation()){
            //將馬達當前輸出的模擬電壓輸入至飛輪物理模擬器中
            m_topFlywheelSim.setInputVoltage(m_topSimState.getMotorVoltage());
            m_bottomFlywheelSim.setInputVoltage(m_bottomSimState.getMotorVoltage());
            //進行20ms得物理步進模擬
            m_topFlywheelSim.update(0.02);
            m_bottomFlywheelSim.update(0.02);
            //獲取物理模你產生的速度(RPM)，轉換成RPS後同步回馬達編碼器
            double topSimRPS = m_topFlywheelSim.getAngularVelocityRadPerSec() / 60.0;
            double bottomSimRPS = m_bottomFlywheelSim.getAngularVelocityRadPerSec() / 60.0;
            // 同步回馬達編碼器
            m_topSimState.setRotorVelocity(topSimRPS);
            m_bottomSimState.setRotorVelocity(bottomSimRPS);
            //設定模型的電池電壓(供編碼器計算精確數值)
            m_topSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            m_bottomSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        }
    }

}