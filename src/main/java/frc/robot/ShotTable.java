package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * 距離 -> 仰角 (Hood 圈數) 與發射輪基準轉速 (RPS) 的對照表。
 *
 * 目標孔的高度不同，所以每一排各有一張表：row 0 = 最上排小孔，row 1~3 = 大孔由上到下。
 * 兩排之間距離相同時，需要的仰角和轉速不一樣，要分開量。
 *
 * 【TODO：實測後填表】
 * 現在每一排只放兩個點，數值都是目前手動操作的預設值（Hood 預設高位、Shooter 預設轉速），
 * 因此機器人行為和手動按鍵一樣，不是量測過的結果。
 * 填法：在固定點發射，記下「距離 (m)、能進球的 Hood 圈數、Shooter RPS」，加進對應那一排的 put(...)。
 * 距離看儀表板／SmartDashboard 的 AutoShoot/Distance m（AutoShoot/Distance Source 說明來源：
 * 有深度相機時是視覺的 dist_m，一般 USB 相機則是里程計推算的值）。填表與實際射擊要用同一種距離來源。
 * 點越多、涵蓋範圍越廣越準，表外會取最近端的值。
 */
public final class ShotTable {
    public record Shot(double hoodRotations, double shooterRps) {}

    private static final int kRows = 4;
    private static final InterpolatingDoubleTreeMap[] kHood = new InterpolatingDoubleTreeMap[kRows];
    private static final InterpolatingDoubleTreeMap[] kRps = new InterpolatingDoubleTreeMap[kRows];

    static {
        for (int r = 0; r < kRows; r++) {
            kHood[r] = new InterpolatingDoubleTreeMap();
            kRps[r] = new InterpolatingDoubleTreeMap();
            // 佔位資料（見上方 TODO）：距離 1.0 m 與 5.0 m 各一點，數值為現行手動預設
            put(r, 1.0, HoodConstants.kHoodPresetHighGoalRotations, ShooterConstants.kShooterDefaultRPS);
            put(r, 5.0, HoodConstants.kHoodPresetHighGoalRotations, ShooterConstants.kShooterDefaultRPS);
        }
    }

    private ShotTable() {}

    private static void put(int row, double distM, double hoodRot, double rps) {
        kHood[row].put(distM, hoodRot);
        kRps[row].put(distM, rps);
    }

    /** @param row 0 = 小孔排，1~3 = 大孔三排  @param distM 相機到目標的水平距離 */
    public static Shot lookup(int row, double distM) {
        int r = Math.max(0, Math.min(kRows - 1, row));
        return new Shot(kHood[r].get(distM), kRps[r].get(distM));
    }
}
