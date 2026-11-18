package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Run: ./gradlew :TeamCode:testDebugUnitTest */
public class BallCounterTest {
    private static final double FAR = 20, NEAR = 2;
    private static final double BETWEEN = (BallCounter.BALL_CM + BallCounter.CLEAR_CM) / 2;

    private static void pass(BallCounter b, boolean reversing) {
        b.update(NEAR, reversing);
        b.update(FAR, reversing);
    }

    @Test
    public void eachBallCountsOnceToFull() {
        BallCounter b = new BallCounter(0);
        for (int i = 0; i < BallCounter.LIMIT; i++) {
            assertFalse(b.full());
            b.update(NEAR, false);
            b.update(NEAR, false);   // still in front: no double count
            b.update(FAR, false);
        }
        assertEquals(BallCounter.LIMIT, b.count());
        assertTrue(b.full());
    }

    @Test
    public void jitterBetweenThresholdsDoesNotRecount() {
        BallCounter b = new BallCounter(0);
        b.update(NEAR, false);
        b.update(BETWEEN, false);
        b.update(NEAR, false);
        assertEquals(1, b.count());
        // and a ball that only reaches the gap isn't counted
        BallCounter c = new BallCounter(0);
        c.update(BETWEEN, false);
        assertEquals(0, c.count());
    }

    @Test
    public void reversingCountsDownAndNeverBelowZero() {
        BallCounter b = new BallCounter(1);
        pass(b, true);
        pass(b, true);
        assertEquals(0, b.count());
    }

    @Test
    public void setOverrides() {
        BallCounter b = new BallCounter(BallCounter.LIMIT);
        b.set(0);
        assertEquals(0, b.count());
        b.set(-3);
        assertEquals(0, b.count());
    }
}
