package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Run: ./gradlew :TeamCode:testDebugUnitTest */
public class BallCounterTest {

    private static void pass(BallCounter b, boolean reversing) {
        b.update(true, reversing);
        b.update(false, reversing);
    }

    @Test
    public void eachBallCountsOnceToFull() {
        BallCounter b = new BallCounter(0);
        for (int i = 0; i < BallCounter.LIMIT; i++) {
            assertFalse(b.full());
            b.update(true, false);
            b.update(true, false);   // still in front: no double count
            b.update(false, false);
        }
        assertEquals(BallCounter.LIMIT, b.count());
        assertTrue(b.full());
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
