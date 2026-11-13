package frc.robot;

public final class Constants {
    
    // ==========================================
    // Shooter (發射器/射球機構) 參數
    // ==========================================
    public final class ShooterConstants {
        // 馬達 CAN ID 設定
        public static final int kTopShooterId = 51;              // 上發射輪 TalonFX 馬達 CAN ID
        public static final int kBottomShooterId = 52;           // 下發射輪 TalonFX 馬達 CAN ID

        // 馬達轉向設定 (雙輪夾球發射時，上下輪旋轉方向通常相反)
        public static final boolean kTopShooterInverted = true;    // 上發射輪是否反向 (預設 false: 逆時針正轉)
        public static final boolean kBottomShooterInverted = false;  // 下發射輪是否反向 (預設 true: 順時針反轉)

        // 控制物件預設參數 (一般初始化為 0 即可)
        public static final double kShooterVelocityVoltage = 0; // 閉環速度控制控制器的初始化預設值
        public static final double kShooterDutyCycleOut = 0;    // 開環百分比控制控制器的初始化預設值

        public static final double kShooterDefaultRPS = 90.0; // 發射輪的預設轉速 (RPS)，用於測試與調整 PID 參數
        public static final double kShooterDefaultSpinFactor = 0.15; // 發射輪的預設旋轉因子 (0~1)，用於測試與調整 PID 參數

        //用於發佈到圖表的目標轉速追蹤變數
        public static final double kTopShooterTargetRPS = 0.0;     // 目標轉速 (RPS)   
        public static final double kBottomShooterTargetRPS = 0.0;  // 目標轉速 (RPS)   

        // 速度控制 PID & 前饋參數 (Velocity PID & Feedforward)
        public static final double kShooterkP = 0.3;          // 比例參數：數值越大，馬達達到目標速度越快
        public static final double kShooterkI = 0.0;            // 積分參數：用於消除靜態誤差，在速度控制中通常設為 0 以防衝過頭
        public static final double kShooterkD = 0.0;          // 微分參數：防震、防止轉速衝過頭
        public static final double kShooterkV = 0.096;           // 速度前饋：極為重要！根據馬達最大轉速與電壓估算 (12V / 最大轉速 RPS)
        public static final double kShooterkS = 0.0;            // 靜態前饋：用於克服摩擦力，通常設為 0

        public static final double kShooterCruiseVelocity = 120.0; // 巡航轉速 (RPS)：馬達達到目標速度後的穩定轉速
        public static final double kShooterAcceleration = 200.0;  // 最大加速度 (RPS/s)：馬達加速到巡航轉速的快慢
        public static final double kShooterJerk = 500.0;            // 加加速度 (Jerk)：限制加速度變化的平滑度 (0 代表不限制)

        public static final double kShooterVelocityToleranceRPS = 0.5; // 速度容忍度 (RPS)：當馬達轉速與目標轉速差距小於此值時，視為已達到目標速度

        // Stator Current Limit (定子/馬達端限流)：保護馬達與限制最大瞬間扭力
        public static final double kShooterStatorCurrentLimit = 60.0;       // 定子電流上限 (A)，防止線圈過熱燒毀
        public static final boolean kShooterStatorCurrentLimitEnable = true; // 是否啟用定子電流限制

        // Supply Current Limit (電源/電池端限流)：保護電池防降壓 (Brownout)
        public static final double kShooterSupplyCurrentLimit = 40.0;       // 主電流限制 (A)：容許瞬間加速抽電的最高值
        public static final double kShooterSupplyCurrentLowerLimit = 40.0;  // 次電流限制 (A)：若持續限流，會降到此數值以防止長期過熱
        public static final double kShooterSupplyCurrentLowerTime = 1.0;     // 時間閥值 (秒)：卡在主限制超過 1 秒後，才降到次限制
        public static final boolean kShooterSupplyCurrentLimitEnable = true;  // 是否啟用電源端電流限制

        // 電壓閉環斜率 (Voltage Ramp Rate)
        public static final double kShooterVoltageClosedLoopRampPeriod = 0.1; // 電壓從 0V 升到 12V 最快所需的秒數，能平滑加速、保護機構

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
        public static final double kTurretNudgeRotations = 1.0 / 360.0;  // 十字鍵每按一下的微調量 (1 度)
        public static final double kTurretPositionToleranceRotations = 0.005; // 轉塔位置容忍度 (圈數，約 1.8 度)：判斷是否瞄準完成

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
        public static final double kTurretMotionMagicJerk = 0;          // 加加速度 (Jerk)：限制加速度變化的平滑度 (0 代表不限制)

        // Stator Current Limit (定子限流)：防推撞或撞擊極限位置時毀壞機構
        public static final double kTurretStatorCurrentLimit = 40.0;        // 定子電流上限 (A)
        public static final boolean kTurretStatorCurrentLimitEnable = true; // 是否啟用定子電流限制

        // Supply Current Limit (電源端限流)：防止卡死堵轉時將電池電量抽乾
        public static final double kTurretSupplyCurrentLimit = 35.0;        // 主電流限制 (A)
        public static final double kTurretSupplyCurrentLowerLimit = 30.0;   // 次電流限制 (A)
        public static final double kTurretSupplyCurrentLowerTime = 1.0;     // 時間閥值 (秒)
        public static final boolean kTurretSupplyCurrentLimitEnable = true;  // 是否啟用電源端電流限制

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
        public static final double kHoodPresetHighGoalRotations = 1.0 / 360.0; // 仰角高目標位置 (圈數)
        public static final double kHoodStartPositionRotations = 40.0 / 360.0; // 開電 / 初始化時，仰角當下位置視為這個圈數（= 最高角度 = 前端軟體限位）
        public static final double kHoodNudgeRotations = 3.0 / 360.0;  // 十字鍵每按一下的微調量 (3 度)
        public static final double kHoodNudgeCooldownSec = 0.3;         // 微調後的冷卻時間 (秒)：冷卻期間按鍵無效
        public static final double kHoodPositionToleranceRotations = 0.005; // 仰角位置容忍度 (圈數)：當仰角位置與目標位置差距小於此值時，視為已達到目標位置

        // 位置控制 PID 參數
        public static final double kHoodkP = 100.0;            // 比例參數：控制仰角反應的速度
        public static final double kHoodkI = 0.0;              // 積分參數
        public static final double kHoodkD = 0.2; 
        public static final double kHoodkS = 0.2;            // 靜態前饋：通常設為 0
        public static final double kHoodkV = 0.2;            // 速度前饋：通常設為 0
        
        public static final boolean kHoodForwardSoftLimitEnable = true; // 是否啟用軟體極限開關，防止仰角旋轉超過物理極限
        public static final double kHoodForwardSoftLimitThreshold = 40.0 / 360.0; // 前端軟體極限閾值（最高 40°）
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

    }

    // ==========================================
    // Feeder (進料/分球機構) 參數
    // ==========================================
    public final class FeederConstants {
        // 馬達 CAN ID 設定
        public static final int kFeederId = 55;                  // 進料分球 TalonFX 馬達 CAN ID

        // 控制物件預設參數
        public static final double kFeederGearRatio = 1.0;      // 分球機構與馬達的齒輪比 (假設為 12:1)
        public static final double kFeederMotionMagicVoltage = 0; // Motion Magic 控制器初始化預設值

        // 機構物理參數
        // 假設機構與馬達之間的齒輪比為 12:1 (馬達轉 12 圈，分球盤轉 1 圈)
        // 分球盤轉 60 度相當於 1/6 圈，因此馬達每次步進需要轉動 12 * (1.0 / 6.0) = 2.0 圈
        public static final double kFeederRotationPerStep = (120.0 / 360.0); // 每次進料按鈕按下的目標步進圈數
        // 卡球偵測與自動退回重試
        public static final double kFeederJamCurrentAmps = 20.0;   // 定子電流超過此值（A）並持續一小段時間，就判定卡球
        public static final double kFeederJamTimeSec = 0.1;       // 電流過大需持續多久（秒）才算卡球（避免啟動瞬間誤判）
        public static final double kFeederWhineCurrentAmps = 15.0; // 待機中「馬達叫」的電流門檻（A）：電流大於此值且幾乎不轉
        public static final double kFeederWhineTimeSec = 0.2;     // 「叫」需持續多久（秒）才自動解卡
        public static final double kFeederStallVelocityRPS = 0.1; // 轉速低於此值（RPS）且還沒到位，視為「卡住不動」
        public static final double kFeederStallTimeSec = 0.15;     // 卡住不動需持續多久（秒）才算卡球
        public static final double kFeederJamReverseDuty = 0.35;   // 卡球時開環反轉的功率（0~1），要夠力才推得開球
        public static final double kFeederJamReverseSec = 0.5;     // 卡球時開環反轉的最長時間（秒）：實際上退到預設位置就會停
        public static final double kFeederStepTimeoutSec = 0.8;   // 單次步進最長時間（秒），逾時沒到位也視為卡球
        public static final double kFeederRecoverTimeoutSec = 0.8; // 退回預設位置的最長時間（秒）
        public static final int kFeederJamMaxRetries = 2;          // 卡球後最多重試幾次（用完就退回並放棄）
        public static final double kFeederCooldownSec = 0.1; // 每次分球結束後的冷卻時間 (秒)：冷卻期間按鍵無效
        public static final double kFeederPositionToleranceRotations = 0.005; // 分球機構位置容忍度 (圈數)：當分球盤位置與目標位置差距小於此值時，視為已達到目標位置

        // 位置控制 PID 參數
        public static final double kFeederkP = 15.0;            // 比例參數：控制分球旋轉的精準度與反應速度
        public static final double kFeederkI = 0.0;            // 積分參數
        public static final double kFeederkD = 0.0;             // 微分參數
        public static final double kFeederkV = 0.0;            // 速度前饋：通常設為 0
        public static final double kFeederkS = 0.12;            // 靜態前饋：用於克服摩擦力，通常設為 0

        // Motion Magic 運動軌跡控制參數
        public static final double kFeederMotionMagicCruiseVelocity = 2; // 巡航轉速 (RPS)
        public static final double kFeederMotionMagicAcceleration = 4;    // 最大加速度 (RPS/s)：120° 約 0.6 秒轉完
        public static final double kFeederMotionMagicJerk = 0;            // 加加速度 (Jerk)

        // Stator Current Limit (定子限流)：非常重要！當進料卡球堵轉時，限制最大扭力避免把球夾碎或損壞機構
        public static final double kFeederStatorCurrentLimit = 40.0;        // 定子電流上限 (A)
        public static final boolean kFeederStatorCurrentLimitEnable = true; // 是否啟用定子限流

        // Supply Current Limit (電源限流)
        public static final double kFeederSupplyCurrentLimit = 35.0;        // 主電流限制 (A)
        public static final double kFeederSupplyCurrentLowerLimit = 30.0;   // 次電流限制 (A)
        public static final double kFeederSupplyCurrentLowerTime = 1.0;     // 時間閥值 (秒)
        public static final boolean kFeederSupplyCurrentLimitEnable = true;  // 是否啟用電源限流

    }

    // ==========================================
    // Orbit (導軌/輸送機構) 參數
    // ==========================================
    public final class OrbitConstants {
        // 馬達 CAN ID 設定
        public static final int kOrbitMotorID = 56;                  // Orbit 主馬達 TalonFX (Kraken X44) CAN ID
        public static final int kOrbitFollowerMotorID = 57;          // Orbit 第二顆馬達 CAN ID（跟隨主馬達，兩顆一起動）
        public static final boolean kOrbitFollowerOpposed = true;   // 第二顆是否與主馬達反向（兩顆面對面安裝時設 true）

        // 控制物件預設參數
        public static final double kOrbitGearRatio = 1.0;           // Orbit 與馬達的齒輪比 (若直接驅動則為 1:1)
        public static final double kOrbitMotionMagicVoltage = 0;    // Motion Magic 控制器初始化預設值

        public static final boolean kOrbitMotorInverted = true;    // Orbit 馬達是否反向旋轉 (視機構設計而定)

        // 預設送球與反轉百分比 (DutyCycle)
        public static final double kOrbitFeedDutyCycle = 0.35;       // 推進送球至 Shooter 的預設功率百分比
        public static final double kOrbitReverseDutyCycle = -0.4;    // 卡球排障反轉時的預設功率百分比

        // 位置與速度控制 PID 參數
        public static final double kOrbitkP = 20.0;                 // 比例參數
        public static final double kOrbitkI = 0.0;                  // 積分參數
        public static final double kOrbitkD = 0.1;                  // 微分參數
        public static final double kOrbitkV = 0.12;                 // 速度前饋
        public static final double kOrbitkS = 0.1;                  // 靜態前饋 (克服摩擦力)

        public static final double kOrbitPositionToleranceRotations = 0.01; // 位置容忍度 (圈數)

        // Motion Magic 運動軌跡控制參數 (用於平滑位置控制)
        public static final double kOrbitMotionMagicCruiseVelocity = 20.0; // 巡航轉速 (RPS)
        public static final double kOrbitMotionMagicAcceleration = 40.0;   // 最大加速度 (RPS/s)
        public static final double kOrbitMotionMagicJerk = 0.0;            // 加加速度 (Jerk)

        // Stator Current Limit (定子限流)：保護馬達防止卡球堵轉燒毀
        public static final double kOrbitStatorCurrentLimit = 40.0;        // 定子電流上限 (A)
        public static final boolean kOrbitStatorCurrentLimitEnable = true; // 是否啟用定子限流

        // Supply Current Limit (電源限流)：防止抽電過大導致電池掉壓
        public static final double kOrbitSupplyCurrentLimit = 35.0;        // 主電流限制 (A)
        public static final double kOrbitSupplyCurrentLowerLimit = 30.0;   // 次電流限制 (A)
        public static final double kOrbitSupplyCurrentLowerTime = 1.0;     // 時間閥值 (秒)
        public static final boolean kOrbitSupplyCurrentLimitEnable = true;  // 是否啟用電源限流

    }

    // ==========================================
    // Vision (Orin Nano 視覺，透過 NetworkTables "Vision" 表傳來)
    // 孔的編號 index = row * 3 + col；row 0 = 最上排 3 個小孔，row 1~3 = 大孔三排，col 0~2 = 由左到右
    // ==========================================
    public final class VisionConstants {
        public static final String kTableName = "Vision";

        public static final int kHoleCount = 12;
        public static final int kFirstBigHoleIndex = 3;      // index 0~2 為小孔，3~11 為大孔

        // 資料新鮮度與平均
        public static final double kFrameStaleSec = 0.5;     // 超過這麼久沒有新的一幀，視為視覺斷線
        public static final double kAverageWindowSec = 0.6;  // 取最近這段時間內的幀，用中位數降低雜訊
        public static final int kMinSamples = 3;             // 窗口內至少要有幾幀有效資料才採用
        public static final int kBufferSize = 40;            // 最多保留幾幀

        // 轉塔已拆除：左右瞄準改由底盤轉向（見 AutoConstants 的 kAim*）

        // 相機（＝發射口，就在相機正下方、朝車頭方向）相對「車身中心」的位置，公尺。前為正、左為正。
        // 機器人 50x50 cm，相機離左邊框 8 cm、離前邊框 11 cm -> 中心前方 25-11=14 cm、左方 25-8=17 cm
        public static final double kCameraForwardM = 0.14;
        public static final double kCameraLeftM = 0.17;

        // 板子鎖定：機器人要停著（低於這些速度），且窗口內每一幀都看到全部 12 個孔
        public static final double kLockMaxSpeedMps = 0.05;
        public static final double kLockMaxOmegaRadPerSec = 0.05;
    }

    // ==========================================
    // Auto (前 60 秒 AI 自主射擊流程)
    // ==========================================
    public final class AutoConstants {
        public static final double kAutoDurationSec = 60.0;      // 規則：開始後前 60 秒須為 AI 自主
        public static final int kMaxBalls = 27;                  // 上半場每隊 27 顆

        public static final int kShotsAtLitSmallHole = 3;        // 亮燈小孔每顆 4 分，最多 3 顆
        public static final int kShotsPerBigHole = 3;            // 大孔每個最多算 3 顆（每顆 2 分）
        // 大孔的射擊順序（index 3~11）。27 顆球扣掉亮燈孔的 3 顆，剩 24 顆，最後一個大孔會因球數用完而略過。
        public static final int[] kBigHoleOrder = {7, 6, 8, 4, 3, 5, 10, 9, 11};

        public static final double kPrepWaitSec = 0.2;           // 設定新目標後，先等這麼久才檢查「到位」，避免讀到上一個目標的舊狀態
        public static final double kReadyTimeoutSec = 3.0;       // 等待車頭角度/仰角/轉速到位的上限

        // ---- 射擊時用底盤轉向瞄準（轉塔已拆除）----
        // 每個目標開始時：目標車頭角度 = 目前車頭角度 - 視覺偏角 (bearing，往右為正) - kAimBiasDeg，底盤原地轉過去並保持。
        public static final double kAimKp = 4.0;                 // 車頭角度比例增益 (rad/s per rad)
        public static final double kAimMaxOmegaRadPerSec = 1.5;  // 瞄準時最大轉速
        public static final double kAimMinOmegaRadPerSec = 0.12; // 克服靜摩擦的最低轉速（誤差小於容許值一半時歸零）
        public static final double kAimToleranceDeg = 1.0;       // 車頭角度誤差在這之內才算瞄準完成
        // 發射口與相機沒對齊時的補償（度，往右為正）：實測球固定往左偏就填正值，讓車頭多往右轉
        public static final double kAimBiasDeg = 0.0;
        public static final double kFeedSec = 0.8;               // 每發 Orbit 送球時間（與手動一致）
        public static final double kCooldownSec = 0.3;           // 每發之間的間隔
        public static final double kWaitForVisionSec = 5.0;      // 開局等視覺上線的上限
        public static final double kWaitForLockSec = 2.0;        // 視覺上線後，等「看到全部孔、鎖定板位」的上限；逾時就用即時視覺

        // ---- 開場移動：從起始框斜向開到射擊定點，到位後相機才開始拍照運算 ----
        // 場地座標（自己定義，單位公尺）：原點 = 機器人作業區後緣中心；+X 朝九宮格；+Y 朝左（面向九宮格時的左手邊）。
        // 依規則圖：作業區 4.0 m 寬 x 5.0 m 深；起始框 0.7 m 寬 x 1.2 m 深，貼在作業區後方左右兩角；
        // 作業區前緣（檔板）到九宮格 2.5 m，所以九宮格板面在 X = 5.0 + 2.5 = 7.5 m、Y = 0（置中）。
        // 起始位置：為了搶快，機器人靠在起始框「靠中線的前角」（站在後方面向九宮格看：
        // 左起始框的右上角、右起始框的左上角），也就是離射擊點最近的那個角。
        // 起始框 X 0 ~ 1.2 m、|Y| 1.3 ~ 2.0 m；機器人 50x50 cm，中心離框的兩邊各 25 cm，再留 kStartMarginM 避免壓線。
        public static final double kStartBoxFrontX = 1.2;        // 起始框前緣
        public static final double kStartBoxInnerAbsY = 1.3;     // 起始框靠中線那一邊：4.0/2 - 0.7 = 1.3 m
        public static final double kRobotHalfM = 0.25;
        public static final double kStartMarginM = 0.02;
        public static final double kStartCenterX = kStartBoxFrontX - kRobotHalfM - kStartMarginM;         // 0.93 m
        public static final double kStartCenterAbsY = kStartBoxInnerAbsY + kRobotHalfM + kStartMarginM;  // 1.57 m
        public static final double kBoardX = 7.5;                // 九宮格板面 X（視覺沒有深度時，用里程計推算到板面的距離）

        // 射擊定點（與起始邊對稱，兩邊都開到這一點）。Y = 0 即正對九宮格中線；X 決定射擊距離。
        // 【待實測決定】距離板面 = kBoardX - kShootX；預設 4.0 m。ShotTable 填表時用的就是這個距離附近。
        public static final double kShootX = 3.5;
        public static final double kShootY = 0.0;

        public static final double kDriveMaxSpeedMps = 1.5;      // 最高速度（底盤極速 2.0 m/s，留餘裕）
        public static final double kDriveMinSpeedMps = 0.15;     // 接近終點的最低速度，避免停在容許誤差外
        public static final double kDriveMaxAccelMps2 = 2.0;     // 加速與減速度
        public static final double kDriveToleranceMeters = 0.05; // 到達定點的容許誤差
        public static final double kDriveHeadingKp = 4.0;        // 車頭保持朝向九宮格的比例增益 (rad/s per rad)
        public static final double kDriveMaxOmegaRadPerSec = 2.0;
        public static final double kDriveTimeoutSec = 8.0;       // 逾時就在原地開始射擊
        public static final double kSettleSec = 0.3;             // 到位後等車身停穩，再開相機
    }
}
