package org.firstinspires.ftc.teamcode.opmodes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Run: ./gradlew :TeamCode:testDebugUnitTest */
public class ShooterBenchTest {
    private static final double EPS = 1e-9;
    private static final double TARGET = 3000.0;

    private static ShooterBench.RpmStats spunUp() {
        ShooterBench.RpmStats s = new ShooterBench.RpmStats();
        s.start(0.0);
        s.update(0.2, 1500, TARGET);
        s.update(0.6, 2950, TARGET); // 98.3%: up to speed
        return s;
    }

    @Test
    public void rpmFrom28CountsPerRev() {
        assertEquals(6000.0, ShooterBench.toRpm(2800.0), EPS);
    }

    @Test
    public void spinUpIsTimeToWithinTwoPercent() {
        ShooterBench.RpmStats s = spunUp();
        assertEquals(0.6, s.spinUpSec, EPS);
        assertTrue(Double.isNaN(s.recoverySec));
    }

    @Test
    public void burstRecordsMinAndRecoveryFromLowestPoint() {
        ShooterBench.RpmStats s = spunUp();
        s.update(1.0, 2700, TARGET);
        s.update(1.1, 2500, TARGET); // lowest point
        s.update(1.3, 2800, TARGET);
        s.update(1.5, 2960, TARGET); // back within 2%
        s.update(1.7, 2980, TARGET);
        assertEquals(2500, s.minRpm, EPS);
        assertEquals(0.4, s.recoverySec, EPS);
    }

    @Test
    public void resetBurstClearsMinAndRecovery() {
        ShooterBench.RpmStats s = spunUp();
        s.update(1.0, 2500, TARGET);
        s.update(1.4, 2990, TARGET);
        s.resetBurst();
        assertEquals(Double.POSITIVE_INFINITY, s.minRpm, EPS);
        assertTrue(Double.isNaN(s.recoverySec));
        assertEquals(0.6, s.spinUpSec, EPS);
    }

    @Test
    public void faultWhenEncodersDisagree() {
        assertNotNull(ShooterBench.RpmStats.fault(2000, -2000, TARGET, 0.5));
    }

    @Test
    public void faultWhenShaftStallsAfterCheckTime() {
        assertNull(ShooterBench.RpmStats.fault(100, 100, TARGET, 1.0));
        assertNotNull(ShooterBench.RpmStats.fault(100, 100, TARGET, 2.0));
    }

    @Test
    public void noFaultWhenRunningNormally() {
        assertNull(ShooterBench.RpmStats.fault(2990, 2985, TARGET, 3.0));
        // One-motor run: right motor unpowered but its encoder still turns with the shaft.
        assertNull(ShooterBench.RpmStats.fault(2990, 2990, TARGET, 3.0));
    }
}
