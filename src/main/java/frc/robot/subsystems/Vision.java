package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;

/**
 * 讀取 Orin Nano 透過 NetworkTables 發布的視覺結果（"Vision" 表）。這個子系統不控制任何馬達。
 *
 * Orin 每處理完一幀，先寫資料陣列、最後才把 frame_id 加 1；這裡以 frame_id 變動當作「新的一幀」，
 * 收進環狀緩衝，取用時用最近一小段時間內的中位數，降低單幀雜訊與偶發誤判。
 */
public class Vision extends SubsystemBase {
    /** 一個孔的視覺量測（中位數結果）。dist/height 為 NaN 表示該孔沒有深度。 */
    public record Target(double bearingDeg, double pitchDeg, double distM, double heightM, int samples) {}

    private record Frame(double time, double[] bearing, double[] pitch, double[] dist, double[] height, int lit) {}

    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable(VisionConstants.kTableName);
    private final IntegerSubscriber m_frameId = m_table.getIntegerTopic("frame_id").subscribe(-1);
    private final BooleanSubscriber m_gridOk = m_table.getBooleanTopic("grid_ok").subscribe(false);
    private final IntegerSubscriber m_lit = m_table.getIntegerTopic("lit_small").subscribe(-1);
    private final DoubleArraySubscriber m_bearing = m_table.getDoubleArrayTopic("bearing_deg").subscribe(new double[0]);
    private final DoubleArraySubscriber m_pitch = m_table.getDoubleArrayTopic("pitch_deg").subscribe(new double[0]);
    private final DoubleArraySubscriber m_dist = m_table.getDoubleArrayTopic("dist_m").subscribe(new double[0]);
    private final DoubleArraySubscriber m_height = m_table.getDoubleArrayTopic("height_m").subscribe(new double[0]);

    private final ArrayDeque<Frame> m_frames = new ArrayDeque<>();
    private long m_lastFrameId = -1;
    private double m_lastFrameTime = Double.NEGATIVE_INFINITY;
    private boolean m_lastGridOk = false;

    @Override
    public void periodic() {
        long id = m_frameId.get();
        double now = Timer.getFPGATimestamp();
        if (id >= 0 && id != m_lastFrameId) {
            m_lastFrameId = id;
            m_lastFrameTime = now;
            m_lastGridOk = m_gridOk.get();
            m_frames.addLast(new Frame(now, m_bearing.get(), m_pitch.get(), m_dist.get(), m_height.get(), (int) m_lit.get()));
        }
        // 丟掉太舊或超過容量的幀
        while (!m_frames.isEmpty()
                && (m_frames.size() > VisionConstants.kBufferSize
                    || now - m_frames.peekFirst().time() > VisionConstants.kAverageWindowSec * 3)) {
            m_frames.removeFirst();
        }
        SmartDashboard.putBoolean("Vision/Connected", isConnected());
        SmartDashboard.putBoolean("Vision/Grid OK", m_lastGridOk);
        SmartDashboard.putNumber("Vision/Lit Small Col", getLitSmallCol());
    }

    /** 最近 kFrameStaleSec 內有收到新的一幀。 */
    public boolean isConnected() {
        return Timer.getFPGATimestamp() - m_lastFrameTime < VisionConstants.kFrameStaleSec;
    }

    /** 亮燈的小孔欄位 (0~2，由左到右)；沒有穩定結果回傳 -1。以窗口內出現最多次的有效值決定。 */
    public int getLitSmallCol() {
        double now = Timer.getFPGATimestamp();
        int[] votes = new int[3];
        int total = 0;
        for (Frame f : m_frames) {
            if (now - f.time() <= VisionConstants.kAverageWindowSec && f.lit() >= 0 && f.lit() <= 2) {
                votes[f.lit()]++;
                total++;
            }
        }
        if (total < VisionConstants.kMinSamples) {
            return -1;
        }
        int best = 0;
        for (int i = 1; i < 3; i++) {
            if (votes[i] > votes[best]) {
                best = i;
            }
        }
        // 要過半數的幀同意，否則視為不穩定
        return votes[best] * 2 > total ? best : -1;
    }

    /** 取指定孔 (0~11) 在最近窗口內的中位數量測；有效幀不足時回傳 empty。 */
    public Optional<Target> getTarget(int holeIndex) {
        if (holeIndex < 0 || holeIndex >= VisionConstants.kHoleCount || !isConnected()) {
            return Optional.empty();
        }
        double now = Timer.getFPGATimestamp();
        List<Double> b = new ArrayList<>();
        List<Double> p = new ArrayList<>();
        List<Double> d = new ArrayList<>();
        List<Double> h = new ArrayList<>();
        for (Frame f : m_frames) {
            if (now - f.time() > VisionConstants.kAverageWindowSec) {
                continue;
            }
            Double bv = at(f.bearing(), holeIndex);
            Double pv = at(f.pitch(), holeIndex);
            if (bv == null || pv == null) {
                continue; // 角度是必要的
            }
            b.add(bv);
            p.add(pv);
            Double dv = at(f.dist(), holeIndex);
            Double hv = at(f.height(), holeIndex);
            if (dv != null) {
                d.add(dv);
            }
            if (hv != null) {
                h.add(hv);
            }
        }
        if (b.size() < VisionConstants.kMinSamples) {
            return Optional.empty();
        }
        return Optional.of(new Target(median(b), median(p), d.isEmpty() ? Double.NaN : median(d),
                h.isEmpty() ? Double.NaN : median(h), b.size()));
    }

    private static Double at(double[] arr, int i) {
        if (arr == null || i >= arr.length || Double.isNaN(arr[i])) {
            return null;
        }
        return arr[i];
    }

    private static double median(List<Double> v) {
        double[] a = v.stream().mapToDouble(Double::doubleValue).toArray();
        Arrays.sort(a);
        int n = a.length;
        return n % 2 == 1 ? a[n / 2] : 0.5 * (a[n / 2 - 1] + a[n / 2]);
    }
}
