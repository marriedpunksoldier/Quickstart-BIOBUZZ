package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Aiming for the goBILDA gear-driven turret (3208-0004-0001): a positional
 * servo drives the 64T pinion, which turns the 176T ring (2.75:1).
 * <p>
 * The Limelight rides on the turret, so its tx is the aiming error itself.
 * Each loop the turret steps toward the target by part of tx, with:
 * <ul>
 *   <li>Deadband: tx under 0.5 deg is Limelight jitter; hold still.</li>
 *   <li>Slew limit: at most MAX_SLEW_DEG_PER_LOOP of turret motion per loop.</li>
 *   <li>Gain under 1: the camera frame lags the servo, so correcting the full
 *       tx every loop re-applies the same error and overshoots.</li>
 * </ul>
 * "Locked on" means a target is visible and within LOCKED_TX_THRESHOLD_DEG.
 * <p>
 * Ported from the 2025 DECODE version, which assumed a chassis-mounted
 * camera (target = center - tx) and 90 deg of travel.
 */
public class TurretSubsystem {

    public static final double TURRET_CENTER = 0.5;
    public static final double TURRET_MIN = 0.0;
    public static final double TURRET_MAX = 1.0;

    /** 176T ring / 64T pinion, measured from goBILDA's STEP file. */
    public static final double GEAR_RATIO = 176.0 / 64.0;

    /**
     * Servo travel over 0..1. goBILDA servos in standard mode are 300 deg.
     * Calibrate on the robot: command 0 and 1, measure the turret angle, and
     * set this to that angle x GEAR_RATIO.
     */
    public static final double SERVO_RANGE_DEG = 300.0;

    /** Turret degrees across the full 0..1 servo range (~109 deg). */
    public static final double TURRET_RANGE_DEG = SERVO_RANGE_DEG / GEAR_RATIO;

    public static final double TX_DEADBAND_DEG = 0.5;

    /** Same turret speed as 2025 (0.04 x 90 deg); ~180 deg/s at 50 Hz. */
    public static final double MAX_SLEW_DEG_PER_LOOP = 3.6;

    // ponytail: fixed fraction of tx per loop; replace with latency-compensated
    // aiming (Limelight capture timestamp) if it still hunts on the robot.
    public static final double AIM_GAIN = 0.5;

    public static final double LOCKED_TX_THRESHOLD_DEG = 2.0;

    private final Servo servo;
    private double currentPosition = TURRET_CENTER;
    private boolean lockedOn = false;
    private double lastKnownTx = 0.0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "turretGear");
        servo.setPosition(TURRET_CENTER);
    }

    /**
     * Aim the turret toward a target at the given horizontal offset.
     * If the turret turns away from the target, reverse the servo's direction
     * in the constructor (the gear mesh reverses rotation).
     *
     * @param txDegrees horizontal angle of target from Limelight (positive = right of center)
     * @param hasTarget true if Limelight currently has a valid target
     */
    public void aimAtTarget(double txDegrees, boolean hasTarget) {
        if (!hasTarget) {
            // Hold the last position; snapping to center on a dropped frame
            // made the 2025 turret pump.
            lockedOn = false;
            return;
        }
        lastKnownTx = txDegrees;
        currentPosition = nextPosition(currentPosition, txDegrees);
        servo.setPosition(currentPosition);
        lockedOn = Math.abs(txDegrees) < LOCKED_TX_THRESHOLD_DEG;
    }

    /** Next servo position for a turret-mounted camera reading tx. */
    static double nextPosition(double current, double txDegrees) {
        if (Math.abs(txDegrees) < TX_DEADBAND_DEG) return current;
        double stepDeg = AIM_GAIN * txDegrees;
        stepDeg = Math.max(-MAX_SLEW_DEG_PER_LOOP, Math.min(MAX_SLEW_DEG_PER_LOOP, stepDeg));
        return clamp(current - stepDeg / TURRET_RANGE_DEG);
    }

    /** Force turret back to center (used when leaving auto-aim modes). */
    public void center() {
        currentPosition = TURRET_CENTER;
        servo.setPosition(TURRET_CENTER);
        lockedOn = false;
    }

    /** Manually nudge the turret (driver-stick override). */
    public void setPosition(double position) {
        currentPosition = clamp(position);
        servo.setPosition(currentPosition);
        lockedOn = false;
    }

    /** Aimed within LOCKED_TX_THRESHOLD_DEG with a target visible; gate firing on this. */
    public boolean isLockedOn()    { return lockedOn; }
    public double getPosition()    { return currentPosition; }
    public double getLastKnownTx() { return lastKnownTx; }

    /** Turret angle from center in degrees, positive = the way positive tx points. */
    public double getAngleDeg()    { return (TURRET_CENTER - currentPosition) * TURRET_RANGE_DEG; }

    private static double clamp(double p) {
        return Math.max(TURRET_MIN, Math.min(TURRET_MAX, p));
    }
}
