package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

/**
 * Which alliance the robot is on. Define every autonomous pose once in BLUE
 * coordinates and pass it through {@link #pose(Pose)} to get the RED one.
 * <p>
 * The BIOBUZZ field is 180-degree ROTATIONALLY symmetric (Competition Manual V1
 * Figs 9-2, 9-5, 9-17, 11-1), not mirrored left-right, so RED is
 * {@code (W - x, W - y, heading + PI)}. Do not use {@code PoseFactory.mirrorX}
 * or the Visualizer's mirror option: both reflect, and mirrorX also gets the
 * heading wrong.
 * <p>
 * The rotation maps each CELL to a different AprilTag set (red rear 30-33
 * becomes blue audience 38-41), so tag IDs are not rotated by this class.
 */
public enum Alliance {
    BLUE,
    RED;

    /**
     * Field size in inches. 141.5 matches the Pedro Visualizer, where our poses
     * come from; 144 would put every RED pose 2.5 in off. Measure the real
     * field and adjust if our poses stop coming from the Visualizer.
     */
    public static final double FIELD_SIZE = 141.5;

    /** Returns the BLUE pose for BLUE, or its 180-degree rotation for RED. */
    public Pose pose(Pose bluePose) {
        if (this == BLUE) return bluePose;
        return new Pose(
                FIELD_SIZE - bluePose.x(),
                FIELD_SIZE - bluePose.y(),
                bluePose.heading() + Math.PI // Pose normalizes to [0, 2PI)
        );
    }

    /** Rotates a heading alone (radians), e.g. for heading interpolation targets. */
    public double heading(double blueHeadingRadians) {
        return pose(new Pose(0, 0, blueHeadingRadians)).heading();
    }

    /**
     * A degrees PoseFactory that applies this alliance's transform. Drop-in for
     * the {@code PoseFactory.degrees()} in Visualizer-exported code (export with
     * mirroring OFF).
     */
    public PoseFactory poses() {
        return PoseFactory.degrees().map(this::pose);
    }
}
