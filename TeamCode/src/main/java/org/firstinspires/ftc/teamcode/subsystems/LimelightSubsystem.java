package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.Alliance;

import java.util.List;

/**
 * Limelight 3A on the turret, aiming at our own HIVE's upward CELL.
 * <p>
 * Each CELL has a cluster of 4 AprilTags on its underside (manual §9.9):
 * Red HIVE 30-33 (rear) and 34-37 (audience); Blue HIVE 38-41 (audience) and
 * 42-45 (rear). The upward CELL flips on every TIP, so we don't hard-code one:
 * of our HIVE's two CELLs, we aim at whichever sits higher in the frame.
 * <p>
 * Poll once per loop with {@link #update()}; the accessors read that cache.
 * If the Limelight is missing, init still succeeds so the robot can drive.
 */
public class LimelightSubsystem {

    /** Limelight pipeline slot configured as AprilTag 36h11. IDs are filtered here. */
    public static final int PIPELINE = 0;

    private static final long STALE_RESULT_MS = 100;
    static final int TAGS_PER_CELL = 4;

    private final Limelight3A limelight;   // null if init failed
    private final String initError;
    private final int firstTagId;

    private boolean hasTarget = false;
    private double txDegrees = 0.0;
    private int targetCell = -1;

    public LimelightSubsystem(HardwareMap hardwareMap, Alliance alliance) {
        firstTagId = firstHiveTag(alliance);
        Limelight3A ll = null;
        String error = null;
        try {
            ll = hardwareMap.get(Limelight3A.class, "limelight");
            ll.setPollRateHz(100);
            ll.pipelineSwitch(PIPELINE);
            ll.start();
        } catch (RuntimeException e) {
            ll = null;
            error = e.getMessage();
        }
        limelight = ll;
        initError = error;
    }

    /** First tag ID of our HIVE; its two CELLs are the next 8 IDs. */
    static int firstHiveTag(Alliance alliance) {
        return alliance == Alliance.RED ? 30 : 38;
    }

    /** Poll the Limelight once and cache the aim. Call once, at the top of loop(). */
    public void update() {
        hasTarget = false;
        txDegrees = 0.0;
        targetCell = -1;
        if (limelight == null) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > STALE_RESULT_MS) return;
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return;

        int n = tags.size();
        int[] ids = new int[n];
        double[] tx = new double[n];
        double[] ty = new double[n];
        for (int i = 0; i < n; i++) {
            LLResultTypes.FiducialResult tag = tags.get(i);
            ids[i] = tag.getFiducialId();
            tx[i] = tag.getTargetXDegrees();
            ty[i] = tag.getTargetYDegrees();
        }
        double[] aim = aim(ids, tx, ty, firstTagId);
        if (aim == null) return;
        hasTarget = true;
        targetCell = (int) aim[0];
        txDegrees = aim[1];
    }

    /**
     * Pick the higher of our HIVE's two CELLs and return {cell index, mean tx},
     * or null if none of our tags are visible. Other tags (the opponent HIVE)
     * are ignored.
     */
    // ponytail: mean tx of the visible tags approximates the cluster centre. Tags sit
    // at about -6.5, -2.75, +2.75, +6.5 in from centre (setup guide p.18), so one outer
    // pair alone is 4.6 in off and a lone outer tag 6.5 in (CELL opening is 20 in wide).
    // If shots land to one side, fit tx against those per-ID offsets instead.
    static double[] aim(int[] ids, double[] tx, double[] ty, int firstTagId) {
        double[] sumTx = new double[2];
        double[] sumTy = new double[2];
        int[] count = new int[2];
        for (int i = 0; i < ids.length; i++) {
            int offset = ids[i] - firstTagId;
            if (offset < 0 || offset >= 2 * TAGS_PER_CELL) continue;
            int cell = offset / TAGS_PER_CELL;
            sumTx[cell] += tx[i];
            sumTy[cell] += ty[i];
            count[cell]++;
        }
        int best = -1;
        for (int c = 0; c < 2; c++) {
            if (count[c] == 0) continue;
            if (best < 0 || sumTy[c] / count[c] > sumTy[best] / count[best]) best = c;
        }
        return best < 0 ? null : new double[] {best, sumTx[best] / count[best]};
    }

    public boolean isConnected()    { return limelight != null; }
    public String getInitError()    { return initError; }
    public boolean hasTarget()      { return hasTarget; }
    public double getTxDegrees()    { return txDegrees; }

    /** 0 or 1 within our HIVE, or -1 with no target. */
    public int getTargetCell()      { return targetCell; }

    /** Tag range of the targeted CELL, e.g. "30-33", for telemetry. */
    public String getTargetTags() {
        if (targetCell < 0) return "none";
        int first = firstTagId + targetCell * TAGS_PER_CELL;
        return first + "-" + (first + TAGS_PER_CELL - 1);
    }
}
