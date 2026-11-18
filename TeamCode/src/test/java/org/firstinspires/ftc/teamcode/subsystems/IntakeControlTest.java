package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/** Run: ./gradlew :TeamCode:testDebugUnitTest */
public class IntakeControlTest {
    private static final double EPS = 1e-9;
    private static final double DT = 0.02;   // 50 Hz loop

    /** Run {@code c} stalled (speed 0) from {@code t} for {@code sec}; returns the end time. */
    private static double stall(IntakeControl c, double t, double sec) {
        for (double end = t + sec; t < end - 1e-9; t += DT) c.update(t, true, false, false, 0);
        return t;
    }

    @Test
    public void runsInWhenFree() {
        IntakeControl c = new IntakeControl();
        assertEquals(IntakeControl.IN_POWER, c.update(0, true, false, false, 0.8), EPS);
        assertEquals(IntakeControl.State.IN, c.state());
    }

    @Test
    public void briefDipIsNotAJam() {
        IntakeControl c = new IntakeControl();
        double t = stall(c, 0, IntakeControl.JAM_SEC - 2 * DT);
        assertEquals(IntakeControl.IN_POWER, c.update(t, true, false, false, 0.8), EPS);
        assertEquals(IntakeControl.State.IN, c.state());
    }

    @Test
    public void stallReversesThenRetries() {
        IntakeControl c = new IntakeControl();
        double t = stall(c, 0, IntakeControl.JAM_SEC + DT);
        assertEquals(IntakeControl.State.UNJAM, c.state());
        assertEquals(IntakeControl.UNJAM_POWER, c.update(t, true, false, false, -0.5), EPS);
        t += IntakeControl.UNJAM_SEC;
        assertEquals(IntakeControl.IN_POWER, c.update(t, true, false, false, -0.5), EPS);
        assertEquals(IntakeControl.State.IN, c.state());
    }

    @Test
    public void givesUpAfterMaxTriesUntilReleased() {
        IntakeControl c = new IntakeControl();
        double t = 0;
        for (int i = 0; i < IntakeControl.MAX_UNJAMS; i++) {
            t = stall(c, t, IntakeControl.JAM_SEC + DT);
            assertEquals(IntakeControl.State.UNJAM, c.state());
            t = stall(c, t, IntakeControl.UNJAM_SEC + DT);
        }
        t = stall(c, t, IntakeControl.JAM_SEC + DT);
        assertEquals(IntakeControl.State.JAMMED, c.state());
        assertEquals(0, c.update(t, true, false, false, 1.0), EPS);
        c.update(t + DT, false, false, false, 0);   // driver lets go
        assertEquals(IntakeControl.IN_POWER, c.update(t + 2 * DT, true, false, false, 1.0), EPS);
    }

    @Test
    public void freeRunningForgetsOldJams() {
        IntakeControl c = new IntakeControl();
        double t = 0;
        for (int i = 0; i < IntakeControl.MAX_UNJAMS; i++) {
            t = stall(c, t, IntakeControl.JAM_SEC + DT);
            t = stall(c, t, IntakeControl.UNJAM_SEC + DT);
            for (double end = t + IntakeControl.CLEAR_SEC + DT; t < end; t += DT) c.update(t, true, false, false, 1.0);
        }
        stall(c, t, IntakeControl.JAM_SEC + DT);
        assertEquals(IntakeControl.State.UNJAM, c.state());
    }

    @Test
    public void fullStopsIntakeButNotOuttake() {
        IntakeControl c = new IntakeControl();
        assertEquals(0, c.update(0, true, false, true, 0), EPS);
        assertEquals(IntakeControl.State.FULL, c.state());
        // a long time "stalled" while full is not a jam
        assertEquals(0, c.update(5, true, false, true, 0), EPS);
        assertEquals(IntakeControl.OUT_POWER, c.update(5.02, true, true, true, 0), EPS);
    }

    @Test
    public void releasedIsOff() {
        assertEquals(0, new IntakeControl().update(0, false, false, false, 0), EPS);
    }
}
