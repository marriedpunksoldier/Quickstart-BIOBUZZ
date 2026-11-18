package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Counts balls aboard from the mouth sensor (goBILDA 3124-0001-0001 laser
 * distance sensor in digital mode, under the ramp just past the roller): each
 * ball passing it adds one, or takes one away if the roller is running
 * backwards. Shots don't come off automatically yet: call {@link #set} after a
 * volley or a FLOWER pull (4 at once, touching, so the sensor can't split them),
 * and the drivers fix miscounts by hand. No SDK types, so it runs on a plain JVM.
 */
public class BallCounter {

    /** Robots may hold up to 4; repeated 5+ is likely STRATEGIC (TU05). */
    public static final int LIMIT = 4;

    private int count;
    private boolean seen = false;

    public BallCounter(int start) {
        count = Math.max(0, start);
    }

    /**
     * Call every loop. {@code ballAtSensor} is the sensor's digital output: its
     * own pot sets the trigger distance, so there's no threshold here.
     */
    public void update(boolean ballAtSensor, boolean reversing) {
        if (ballAtSensor && !seen) count = Math.max(0, count + (reversing ? -1 : 1));
        seen = ballAtSensor;
    }

    public void set(int n) {
        count = Math.max(0, n);
    }

    public int count() {
        return count;
    }

    public boolean full() {
        return count >= LIMIT;
    }
}
