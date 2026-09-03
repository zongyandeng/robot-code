package frc.robot;

public final class Constants {
    
    // ==========================================
    // Shooter (發射器/射球機構) 參數
    // ==========================================
    public final class ShooterConstants {
        // 馬達 CAN ID 設定
        public static final int kTopShooterId = 51;              // 上發射輪 TalonFX 馬達 CAN ID
        public static final int kBottomShooterId = 52;           // 下發射輪 TalonFX 馬達 CAN ID
        
        // 控制物件預設參數 (一般初始化為 0 即可)
        public static final double kShooterVelocityVoltage = 0; // 閉環速度控制控制器的初始化預設值
        public static final double kShooterDutyCycleOut = 0;    // 開環百分比控制控制器的初始化預設值

        public static final double kShooterDefaultRPS = 60.0; // 發射輪的預設轉速 (RPS)，用於測試與調整 PID 參數
        public static final double kShooterDefaultSpinFactor = 0.15; // 發射輪的預設旋轉因子 (0~1)，用於測試與調整 PID 參數

        //用於發佈到圖表的目標轉速追蹤變數
        public static final double kTopShooterTargetRPS = 0.0;     // 目標轉速 (RPS)   
        public static final double kBottomShooterTargetRPS = 0.0;  // 目標轉速 (RPS)   

        // 速度控制 PID & 前饋參數 (Velocity PID & Feedforward)
        public static final double kShooterkP = 0.12;           // 比例參數：數值越大，馬達達到目標速度越快
        public static final double kShooterkI = 0.0;            // 積分參數：用於消除靜態誤差，在速度控制中通常設為 0 以防衝過頭
        public static final double kShooterkD = 0.0;          // 微分參數：防震、防止轉速衝過頭
        public static final double kShooterkV = 0.096;           // 速度前饋：極為重要！根據馬達最大轉速與電壓估算 (12V / 最大轉速 RPS)
        public static final double kShooterkS = 0.0;            // 靜態前饋：用於克服摩擦力，通常設為 0

        public static final double kShooterCruiseVelocity = 100.0; // 巡航轉速 (RPS)：馬達達到目標速度後的穩定轉速
        public static final double kShooterAcceleration = 90.0;   // 最大加速度 (RPS/s)：馬達加速到巡航轉速的快慢
        public static final double kShooterJerk = 500.0;            // 加加速度 (Jerk)：限制加速度變化的平滑度 (0 代表不限制)

        public static final double kShooterVelocityToleranceRPS = 0.5; // 速度容忍度 (RPS)：當馬達轉速與目標轉速差距小於此值時，視為已達到目標速度

        // Stator Current Limit (定子/馬達端限流)：保護馬達與限制最大瞬間扭力
        public static final double kShooterStatorCurrentLimit = 40.0;        // 定子電流上限 (A)，防止線圈過熱燒毀
        public static final boolean kShooterStatorCurrentLimitEnable = true; // 是否啟用定子電流限制

        // Supply Current Limit (電源/電池端限流)：保護電池防降壓 (Brownout)
        public static final double kShooterSupplyCurrentLimit = 35.0;        // 主電流限制 (A)：容許瞬間加速抽電的最高值
        public static final double kShooterSupplyCurrentLowerLimit = 30.0;   // 次電流限制 (A)：若持續限流，會降到此數值以防止長期過熱
        public static final double kShooterSupplyCurrentLowerTime = 1.0;     // 時間閥值 (秒)：卡在主限制超過 1 秒後，才降到次限制
        public static final boolean kShooterSupplyCurrentLimitEnable = true;  // 是否啟用電源端電流限制

        // 電壓閉環斜率 (Voltage Ramp Rate)
        public static final double kShooterVoltageClosedLoopRampPeriod = 0.25; // 電壓從 0V 升到 12V 最快所需的秒數，能平滑加速、保護機構

        // 發射輪物理參數 (用於模擬)
        public static final double kTopShooterGearRatio = 1.0;      // 發射輪與馬達的齒輪比 (假設為 1:1)
        public static final double kTopShooterFlywheelMOI = 0.0005; // 上發射輪的轉動慣量 (kg*m^2)，用於模擬
        public static final double kBottomShooterGearRatio = 1.0;   // 下發射輪與馬達的齒輪比 (假設為 1:1)
        public static final double kBottomShooterFlywheelMOI = 0.0005; // 下發射輪的轉動慣量 (kg*m^2)，用於模擬
    }

    // ==========================================
    // Turret (轉塔/旋轉瞄準機構) 參數
    // ==========================================
    public final class TurretConstants {
        // 馬達 CAN ID 設定
        public static final int kTurretId = 53;                  // 轉塔 TalonFX 馬達 CAN ID

        // 控制物件預設參數
        public static final double kTurretGearRatio = 10.0;      // 轉塔與馬達的齒輪比 (假設為 1:1)
        public static final double kTurretMotionMagicVoltage = 0; // Motion Magic 控制器初始化預設值

        public static final boolean kTurretMotorInverted = false; // 轉塔馬達是否反向旋轉 (視機構設計而定)

        // 位置控制 PID 參數 (Position PID)
        public static final double kTurretkP = 20.0;            // 比例參數：控制轉塔旋轉到目標位置的反應速度與力道
        public static final double kTurretkI = 0.0;            // 積分參數：通常設為 0
        public static final double kTurretkD = 0.0;  
        public static final double kTurretkV = 0.2;            // 速度前饋：通常設為 0
        public static final double kTurretkS = 0.2;            // 靜態前饋：通常設為 0
        
        public static final boolean kTurretForwardSoftLimitEnable = true; // 是否啟用軟體極限開關，防止轉塔旋轉超過物理極限
        public static final double kTurretForwardSoftLimitThreshold = 0.25; // 前端軟體極限閾值
        public static final boolean kTurretReverseSoftLimitEnable = true; // 是否啟用軟體極限開關，防止轉塔旋轉超過物理極限
        public static final double kTurretReverseSoftLimitThreshold = -0.25; // 後端軟體極限閾值

        // Motion Magic 運動軌跡控制參數 (用於平滑位置控制)
        public static final double kTurretMotionMagicCruiseVelocity = 1.0; // 巡航轉速 (RPS)：轉塔移動時的最大穩定轉速
        public static final double kTurretMotionMagicAcceleration = 2.5;   // 最大加速度 (RPS/s)：轉塔加速到巡航轉速的快慢
        public static final double kTurretdMotionMagicJerk = 0;          // 加加速度 (Jerk)：限制加速度變化的平滑度 (0 代表不限制)

        // Stator Current Limit (定子限流)：防推撞或撞擊極限位置時毀壞機構
        public static final double kTurretStatorCurrentLimit = 40.0;        // 定子電流上限 (A)
        public static final boolean kTurretStatorCurrentLimitEnable = true; // 是否啟用定子電流限制

        // Supply Current Limit (電源端限流)：防止卡死堵轉時將電池電量抽乾
        public static final double kTurretSupplyCurrentLimit = 35.0;        // 主電流限制 (A)
        public static final double kTurretSupplyCurrentLowerLimit = 30.0;   // 次電流限制 (A)
        public static final double kTurretSupplyCurrentLowerTime = 1.0;     // 時間閥值 (秒)
        public static final boolean kTurretSupplyCurrentLimitEnable = true;  // 是否啟用電源端電流限制

        // 轉塔物理參數 (用於模擬)
        public static final double kTurretFlywheelMOI = 0.0005; // 轉塔的轉動慣量 (kg*m^2)，用於模擬
        public static final double kTurretArmLengthMeters = 0.3; // 轉塔旋轉半徑 (公尺)
        public static final double kTurretMaxAngleRads = Math.PI;     // 轉塔最大旋轉角度 (度)
        public static final double kTurretMinAngleRads = -Math.PI;      // 轉塔最小旋轉角度 (度)
    }

    // ==========================================
    // Hood (仰角/發射面板角度調整機構) 參數
    // ==========================================
    public final class HoodConstants {
        // 馬達 CAN ID 設定
        public static final int kHoodId = 54;                    // 仰角調整 TalonFX 馬達 CAN ID

        // 控制物件預設參數
        public static final double kHoodGearRatio = 10.0;        // 仰角與馬達的齒輪比 (假設為 1:1)
        public static final double kHoodMotionMagicVoltage = 0;  // Motion Magic 控制器初始化預設值

        public static final double kHoodStowedPositionRotations = 0.0; // 仰角收納位置 (圈數)
        public static final double kHoodPresetHighGoalRotations = 0.1; // 仰角高目標位置 (圈數)
        public static final double kHoodPositionToleranceRotations = 0.005; // 仰角位置容忍度 (圈數)：當仰角位置與目標位置差距小於此值時，視為已達到目標位置

        // 位置控制 PID 參數
        public static final double kHoodkP = 20.0;              // 比例參數：控制仰角反應的速度
        public static final double kHoodkI = 0.0;              // 積分參數
        public static final double kHoodkD = 0.2; 
        public static final double kHoodkS = 0.2;            // 靜態前饋：通常設為 0
        public static final double kHoodkV = 0.2;            // 速度前饋：通常設為 0
        
        public static final boolean kHoodForwardSoftLimitEnable = true; // 是否啟用軟體極限開關，防止仰角旋轉超過物理極限
        public static final double kHoodForwardSoftLimitThreshold = 0.25; // 前端軟體極限閾值
        public static final boolean kHoodReverseSoftLimitEnable = true; // 是否啟用軟體極限開關，防止仰角旋轉超過物理極限
        public static final double kHoodReverseSoftLimitThreshold = 0.0; // 後端軟體極限

        // Motion Magic 運動軌跡控制參數
        public static final double kHoodMotionMagicCruiseVelocity = 1.0;  // 巡航轉速 (RPS)
        public static final double kHoodMotionMagicAcceleration = 2.5;    // 最大加速度 (RPS/s)
        public static final double kHoodMotionMagicJerk = 10.0;            // 加加速度 (Jerk)

        // Stator Current Limit (定子限流)：防止仰角推到底時將機構頂壞
        public static final double kHoodStatorCurrentLimit = 35.0;        // 定子電流上限 (A)
        public static final boolean kHoodStatorCurrentLimitEnable = true; // 是否啟用定子限制

        // Supply Current Limit (電源限流)
        public static final double kHoodSupplyCurrentLimit = 30.0;        // 主電流限制 (A)
        public static final double kHoodSupplyCurrentLowerLimit = 25.0;   // 次電流限制 (A)
        public static final double kHoodSupplyCurrentLowerTime = 0.5;     // 時間閥值 (秒)
        public static final boolean kHoodSupplyCurrentLimitEnable = true;  // 是否啟用電源限制

        // 仰角物理參數 (用於模擬)
        public static final double kHoodFlywheelMOI = 0.0005; // 仰角的轉動慣量 (kg*m^2)，用於模擬
        public static final double kHoodArmLengthMeters = 0.3; // 仰角面板的長度/半徑 (公尺)
        public static final double kHoodMaxAngleRads = Math.toRadians(72);     // 仰角最大旋轉角度 (弧度)
        public static final double kHoodMinAngleRads = Math.toRadians(0);      // 仰角最小旋轉角度 (弧度)
    }

    // ==========================================
    // Feeder (進料/分球機構) 參數
    // ==========================================
    public final class FeederConstants {
        // 馬達 CAN ID 設定
        public static final int kFeederId = 55;                  // 進料分球 TalonFX 馬達 CAN ID

        // 控制物件預設參數
        public static final double kFeederMotionMagicVoltage = 0; // Motion Magic 控制器初始化預設值

        // 機構物理參數
        // 假設機構與馬達之間的齒輪比為 12:1 (馬達轉 12 圈，分球盤轉 1 圈)
        // 分球盤轉 60 度相當於 1/6 圈，因此馬達每次步進需要轉動 12 * (1.0 / 6.0) = 2.0 圈
        public static final double kFeederRotationPerStep = 1.0 * (60.0 / 360.0); // 每次進料按鈕按下的目標步進圈數

        // 位置控制 PID 參數
        public static final double kFeederkP = 0.0;            // 比例參數：控制分球旋轉的精準度與反應速度
        public static final double kFeederkI = 0.0;            // 積分參數
        public static final double kFeederkD = 0.0;             // 微分參數

        // Motion Magic 運動軌跡控制參數
        public static final double kFeederMotionMagicCruiseVelocity = 15; // 巡航轉速 (RPS)
        public static final double kFeederMotionMagicAcceleration = 30;   // 最大加速度 (RPS/s)
        public static final double kFeederMotionMagicJerk = 0;            // 加加速度 (Jerk)

        // Stator Current Limit (定子限流)：非常重要！當進料卡球堵轉時，限制最大扭力避免把球夾碎或損壞機構
        public static final double kFeederStatorCurrentLimit = 40.0;        // 定子電流上限 (A)
        public static final boolean kFeederStatorCurrentLimitEnable = true; // 是否啟用定子限流

        // Supply Current Limit (電源限流)
        public static final double kFeederSupplyCurrentLimit = 35.0;        // 主電流限制 (A)
        public static final double kFeederSupplyCurrentLowerLimit = 30.0;   // 次電流限制 (A)
        public static final double kFeederSupplyCurrentLowerTime = 1.0;     // 時間閥值 (秒)
        public static final boolean kFeederSupplyCurrentLimitEnable = true;  // 是否啟用電源限流

        // 分球機構物理參數 (用於模擬)
        public static final double kFeederGearRatio = 1.0;      // 分球機構與馬達的齒輪比 (假設為 12:1)
        public static final double kFeederFlywheelMOI = 0.0005; // 分球機構的轉動慣量 (kg*m^2)，用於模擬
    }
}
