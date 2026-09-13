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
}
