package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
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
 *
 * 板子鎖定（Board lock）：機器人停著、相機同時看到全部 12 個孔時，把每個孔換算成「場地座標」記下來
 * （機器人位置 + 車頭角度 + 相機在車上的位置 + 視覺偏角/距離）。之後仰角機構把相機帶高、孔出了畫面也沒關係，
 * 瞄準直接用「目前里程計位置」對「鎖定的孔位置」計算（aimFromLock），並把發射口在車身左前方的偏移算進去。
 * 看到全部孔又停著時會持續更新鎖定；setCameraEnabled() 或 clearLock() 會清掉（例如自主開場重設里程計時）。
 */
public class Vision extends SubsystemBase {
    /** 一個孔的視覺量測（中位數結果）。dist/height 為 NaN 表示該孔沒有深度。 */
    public record Target(double bearingDeg, double pitchDeg, double distM, double heightM, int samples) {}

    private record Frame(double time, double[] bearing, double[] pitch, double[] dist, double[] height, int lit,
            int nBig, int nSmall) {}

    /** 從鎖定的板位算出的瞄準：車頭要轉到的場地角度（rad）、發射口到孔的水平距離（m）。 */
    public record Aim(double headingRad, double distM) {}

    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable(VisionConstants.kTableName);
    private final IntegerSubscriber m_frameId = m_table.getIntegerTopic("frame_id").subscribe(-1);
    private final BooleanSubscriber m_gridOk = m_table.getBooleanTopic("grid_ok").subscribe(false);
    private final IntegerSubscriber m_lit = m_table.getIntegerTopic("lit_small").subscribe(-1);
    private final DoubleArraySubscriber m_bearing = m_table.getDoubleArrayTopic("bearing_deg").subscribe(new double[0]);
    private final DoubleArraySubscriber m_pitch = m_table.getDoubleArrayTopic("pitch_deg").subscribe(new double[0]);
    private final DoubleArraySubscriber m_dist = m_table.getDoubleArrayTopic("dist_m").subscribe(new double[0]);
    private final DoubleArraySubscriber m_height = m_table.getDoubleArrayTopic("height_m").subscribe(new double[0]);
    private final IntegerSubscriber m_nBig = m_table.getIntegerTopic("n_big").subscribe(-1);
    private final IntegerSubscriber m_nSmall = m_table.getIntegerTopic("n_small").subscribe(-1);

    // 板子鎖定（場地座標，公尺）
    private Supplier<Pose2d> m_pose = null;
    private Supplier<ChassisSpeeds> m_speeds = null;
    private boolean m_locked = false;
    private final double[][] m_lockXY = new double[VisionConstants.kHoleCount][2];
    private double m_lockTime = Double.NEGATIVE_INFINITY;

    // roboRIO -> Orin 的開關：false 時 Orin 不拍照、不跑辨識（vision_node.py 訂閱 Vision/enable）
    private final BooleanPublisher m_enablePub = m_table.getBooleanTopic("enable").publish();

    private final ArrayDeque<Frame> m_frames = new ArrayDeque<>();
    private long m_lastFrameId = -1;
    private double m_lastFrameTime = Double.NEGATIVE_INFINITY;
    private boolean m_lastGridOk = false;
    private boolean m_cameraEnabled = true;

    public Vision() {
        m_enablePub.set(true);
    }

    /** 提供機器人位置與速度（場地座標），板子鎖定才會運作。RobotContainer 建立後呼叫一次。 */
    public void setRobotState(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        m_pose = pose;
        m_speeds = speeds;
    }

    /**
     * 開關 Orin 的拍照與辨識。關閉時丟掉所有舊資料；重新開啟後要等到 Orin 送來「新的」一幀才算視覺上線
     * （isConnected() 會先回 false），不會讀到開關之前的舊結果。
     */
    public void setCameraEnabled(boolean enabled) {
        m_cameraEnabled = enabled;
        m_enablePub.set(enabled);
        m_frames.clear();
        m_lastFrameTime = Double.NEGATIVE_INFINITY;
        m_lastFrameId = m_frameId.get(); // 以目前的 frame_id 為基準，只接受之後才出現的新幀
        clearLock();
    }

    public void clearLock() {
        m_locked = false;
        m_lockTime = Double.NEGATIVE_INFINITY;
    }

    public boolean isLocked() {
        return m_locked;
    }

    /** 鎖定的孔在場地上的位置（公尺）；沒鎖定回傳 empty。 */
    public Optional<Translation2d> lockedHole(int index) {
        if (!m_locked || index < 0 || index >= VisionConstants.kHoleCount) {
            return Optional.empty();
        }
        return Optional.of(new Translation2d(m_lockXY[index][0], m_lockXY[index][1]));
    }

    /** 相機朝向相對底盤車頭的偏角（見 VisionConstants.kCameraYawOffsetDeg）。 */
    private static final Rotation2d kCamYaw = Rotation2d.fromDegrees(VisionConstants.kCameraYawOffsetDeg);

    /** 相機（＝發射口，在相機正下方）的場地位置。camHeading = 相機朝向（場地角度），不是底盤車頭。 */
    private static Translation2d cameraPos(Pose2d robot, Rotation2d camHeading) {
        return robot.getTranslation().plus(
                new Translation2d(VisionConstants.kCameraForwardM, VisionConstants.kCameraLeftM).rotateBy(camHeading));
    }

    /** 底盤車頭角度 -> 相機朝向（場地角度）。 */
    public static Rotation2d cameraHeading(Rotation2d robotHeading) {
        return robotHeading.plus(kCamYaw);
    }

    /**
     * 用鎖定的板位算瞄準孔 index 的「底盤車頭角度」與距離。發射口在相機正下方、朝相機方向，
     * 要找的是「讓發射口沿相機方向的直線穿過孔」的相機朝向；原地轉動時發射口位置也會變，用迭代求解，
     * 最後扣掉相機偏角換成底盤車頭角度（AutoShoot 轉向用的是底盤角度）。
     */
    public Optional<Aim> aimFromLock(int index, Pose2d robot) {
        Optional<Translation2d> t = lockedHole(index);
        if (t.isEmpty()) {
            return Optional.empty();
        }
        Translation2d target = t.get();
        double th = Math.atan2(target.getY() - robot.getY(), target.getX() - robot.getX());
        for (int k = 0; k < 6; k++) {                       // th = 相機朝向
            Translation2d cam = cameraPos(robot, new Rotation2d(th));
            th = Math.atan2(target.getY() - cam.getY(), target.getX() - cam.getX());
        }
        Translation2d cam = cameraPos(robot, new Rotation2d(th));
        return Optional.of(new Aim(MathUtil.angleModulus(th - kCamYaw.getRadians()), cam.getDistance(target)));
    }

    /** 停著、且最近窗口內每一幀都看到全部 12 個孔時，把孔換算成場地座標鎖定（每個 periodic 都會更新）。 */
    private void updateLock(double now) {
        if (m_pose == null || !isConnected()) {
            return;
        }
        if (m_speeds != null) {
            ChassisSpeeds v = m_speeds.get();
            if (Math.hypot(v.vxMetersPerSecond, v.vyMetersPerSecond) > VisionConstants.kLockMaxSpeedMps
                    || Math.abs(v.omegaRadiansPerSecond) > VisionConstants.kLockMaxOmegaRadPerSec) {
                return;
            }
        }
        int n = 0;
        for (Frame f : m_frames) {
            if (now - f.time() > VisionConstants.kAverageWindowSec) {
                continue;
            }
            if (f.nBig() != 9 || f.nSmall() != 3) {
                return;                      // 窗口內有任何一幀沒看到全部孔就不鎖
            }
            n++;
        }
        if (n < VisionConstants.kMinSamples) {
            return;
        }
        Pose2d robot = m_pose.get();
        Rotation2d camHeading = cameraHeading(robot.getRotation());
        Translation2d cam = cameraPos(robot, camHeading);
        double[][] xy = new double[VisionConstants.kHoleCount][2];
        for (int i = 0; i < VisionConstants.kHoleCount; i++) {
            Optional<Target> t = getTarget(i);
            if (t.isEmpty() || Double.isNaN(t.get().distM())) {
                return;
            }
            // 偏角（相對相機）往右為正；場地角度逆時針為正 -> 孔的方向 = 相機朝向 - 偏角
            double ang = camHeading.getRadians() - Math.toRadians(t.get().bearingDeg());
            xy[i][0] = cam.getX() + t.get().distM() * Math.cos(ang);
            xy[i][1] = cam.getY() + t.get().distM() * Math.sin(ang);
        }
        for (int i = 0; i < VisionConstants.kHoleCount; i++) {
            m_lockXY[i][0] = xy[i][0];
            m_lockXY[i][1] = xy[i][1];
        }
        m_locked = true;
        m_lockTime = now;
    }

    public boolean isCameraEnabled() {
        return m_cameraEnabled;
    }

    @Override
    public void periodic() {
        long id = m_frameId.get();
        double now = Timer.getFPGATimestamp();
        if (m_cameraEnabled && id >= 0 && id != m_lastFrameId) {
            m_lastFrameId = id;
            m_lastFrameTime = now;
            m_lastGridOk = m_gridOk.get();
            m_frames.addLast(new Frame(now, m_bearing.get(), m_pitch.get(), m_dist.get(), m_height.get(), (int) m_lit.get(),
                    (int) m_nBig.get(), (int) m_nSmall.get()));
        }
        // 丟掉太舊或超過容量的幀
        while (!m_frames.isEmpty()
                && (m_frames.size() > VisionConstants.kBufferSize
                    || now - m_frames.peekFirst().time() > VisionConstants.kAverageWindowSec * 3)) {
            m_frames.removeFirst();
        }
        SmartDashboard.putBoolean("Vision/Camera Enabled", m_cameraEnabled);
        SmartDashboard.putBoolean("Vision/Connected", isConnected());
        SmartDashboard.putBoolean("Vision/Grid OK", m_lastGridOk);
        SmartDashboard.putNumber("Vision/Lit Small Col", getLitSmallCol());

        if (m_cameraEnabled) {
            updateLock(now);
        }
        // 給儀表板的場地圖：鎖定的 12 個孔場地座標 [x0,y0,x1,y1,...]，以及鎖定多久了
        double[] holes = new double[VisionConstants.kHoleCount * 2];
        for (int i = 0; i < VisionConstants.kHoleCount; i++) {
            holes[2 * i] = m_locked ? m_lockXY[i][0] : Double.NaN;
            holes[2 * i + 1] = m_locked ? m_lockXY[i][1] : Double.NaN;
        }
        SmartDashboard.putBoolean("Vision/Locked", m_locked);
        SmartDashboard.putNumber("Vision/Lock Age s", m_locked ? now - m_lockTime : -1);
        SmartDashboard.putNumberArray("Vision/Board Holes", holes);
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
