package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Joystick-conditioning math shared by TeleOp and anything else taking raw
 * stick input.
 * <ul>
 *   <li>Radial deadzone (magnitude, not per-axis) avoids diagonal drift.</li>
 *   <li>Output is 0 at the deadzone edge and ramps to 1 at full deflection.</li>
 *   <li>Squared-with-sign curve for finer slow control.</li>
 * </ul>
 */
public final class DriveUtils {

    public static final double DEFAULT_DEADBAND = 0.05;

    /** Single-axis deadzone with smooth post-deadband scaling. */
    public static double applyDeadzone(double value, double deadband) {
        double abs = Math.abs(value);
        if (abs < deadband) return 0;
        return Math.signum(value) * (abs - deadband) / (1.0 - deadband);
    }

    /** Single-axis deadzone with default deadband. */
    public static double applyDeadzone(double value) {
        return applyDeadzone(value, DEFAULT_DEADBAND);
    }

    /** Radial deadzone + smooth scaling for a 2D stick. Returns {x, y}. */
    public static double[] applyRadialDeadzone(double x, double y, double deadband) {
        double magnitude = Math.hypot(x, y);
        if (magnitude < deadband) return new double[]{0, 0};
        double scale = (magnitude - deadband) / (1.0 - deadband) / magnitude;
        return new double[]{x * scale, y * scale};
    }

    /** Squared-with-sign curve: preserves sign, finer control near zero. */
    public static double squareCurve(double value) {
        return value * Math.abs(value);
    }

    private DriveUtils() {}
}
