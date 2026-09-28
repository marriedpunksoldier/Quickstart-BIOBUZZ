package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/** Run: ./gradlew :TeamCode:testDebugUnitTest */
public class TurretSubsystemTest {
    private static final double EPS = 1e-9;
    private static final double C = TurretSubsystem.TURRET_CENTER;

    @Test
    public void rangeComesFromGearRatio() {
        assertEquals(109.0909, TurretSubsystem.TURRET_RANGE_DEG, 1e-4);
    }

    @Test
    public void jitterInsideDeadbandHolds() {
        assertEquals(0.3, TurretSubsystem.nextPosition(0.3, 0.49), EPS);
        assertEquals(0.3, TurretSubsystem.nextPosition(0.3, -0.49), EPS);
    }

    @Test
    public void stepIsGainTimesTxAndSteersAgainstTx() {
        // tx = +4: step 2 turret deg, position decreases (same sign as 2025).
        double expected = C - 2.0 / TurretSubsystem.TURRET_RANGE_DEG;
        assertEquals(expected, TurretSubsystem.nextPosition(C, 4.0), EPS);
        assertEquals(C + 2.0 / TurretSubsystem.TURRET_RANGE_DEG, TurretSubsystem.nextPosition(C, -4.0), EPS);
    }

    @Test
    public void largeErrorIsSlewLimited() {
        double maxStep = TurretSubsystem.MAX_SLEW_DEG_PER_LOOP / TurretSubsystem.TURRET_RANGE_DEG;
        assertEquals(C - maxStep, TurretSubsystem.nextPosition(C, 40.0), EPS);
    }

    @Test
    public void clampsAtServoLimits() {
        assertEquals(0.0, TurretSubsystem.nextPosition(0.01, 40.0), EPS);
        assertEquals(1.0, TurretSubsystem.nextPosition(0.99, -40.0), EPS);
    }
}
