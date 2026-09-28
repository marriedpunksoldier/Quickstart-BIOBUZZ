package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Run: ./gradlew :TeamCode:testDebugUnitTest */
public class DriveUtilsTest {
    private static final double EPS = 1e-9;

    @Test
    public void deadzoneZeroesSmallInputAndRampsFromEdge() {
        assertEquals(0, DriveUtils.applyDeadzone(0.04), EPS);
        assertEquals(0, DriveUtils.applyDeadzone(0.05), EPS);
        assertEquals(1, DriveUtils.applyDeadzone(1), EPS);
        assertEquals(-1, DriveUtils.applyDeadzone(-1), EPS);
    }

    @Test
    public void radialDeadzoneUsesMagnitudeAndKeepsDirection() {
        // Each axis 0.04 (inside a square deadzone) but magnitude 0.0566 > 0.05.
        double[] xy = DriveUtils.applyRadialDeadzone(0.04, 0.04, 0.05);
        assertEquals(xy[0], xy[1], EPS);
        assertTrue(xy[0] > 0);

        assertArrayEquals(new double[]{0, 0}, DriveUtils.applyRadialDeadzone(0.03, 0.03, 0.05), EPS);
        assertArrayEquals(new double[]{0, 1}, DriveUtils.applyRadialDeadzone(0, 1, 0.05), EPS);
    }

    @Test
    public void squareCurvePreservesSign() {
        assertEquals(0.25, DriveUtils.squareCurve(0.5), EPS);
        assertEquals(-0.25, DriveUtils.squareCurve(-0.5), EPS);
    }

    /**
     * TeleopBase passes driveOrHold an input threshold of exactly 0, which is
     * only safe because conditioning is exactly 0 at rest and non-zero as soon
     * as the stick clears the deadzone. If that stops holding, TeleOp either
     * never latches its pose or ignores slow creeps.
     */
    @Test
    public void conditionedStickIsExactlyZeroAtRestAndNonZeroOnceOutsideDeadzone() {
        assertEquals(0, condition(0), 0);
        assertEquals(0, condition(DriveUtils.DEFAULT_DEADBAND), 0);

        // A 20% creep is far below Pedro's stock 0.1 threshold but must still read as input.
        assertTrue(condition(0.20) > 0);
        assertTrue(condition(0.20) < 0.1);
        assertTrue(condition(-0.20) < 0);
    }

    /** Mirrors TeleopBase.drivePowers(): radial deadzone, then square curve. */
    private static double condition(double rawStick) {
        double[] xy = DriveUtils.applyRadialDeadzone(rawStick, 0, DriveUtils.DEFAULT_DEADBAND);
        return DriveUtils.squareCurve(xy[0]);
    }
}
