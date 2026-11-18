package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Counts balls aboard from one distance sensor at the intake mouth: each ball
 * passing it adds one, or takes one away if the roller is running backwards.
 * Shots don't come off automatically yet: call {@link #set} after a volley, and
 * the drivers fix miscounts by hand. No SDK types, so it runs on a plain JVM.
 */
public class BallCounter {

    /** Robots may hold up to 4; repeated 5+ is likely STRATEGIC (TU05). */
    public static final int LIMIT = 4;

    /** A ball is in front of the sensor under this distance... (MEASURE on the robot) */
    static final double BALL_CM = 5.0;
    /** ...and gone again over this one. The gap stops one ball counting twice. */
    static final double CLEAR_CM = 7.0;

    private int count;
    private boolean seen = false;

    public BallCounter(int start) {
        count = Math.max(0, start);
    }

    /** Call every loop with the sensor reading and whether the roller is running backwards. */
    public void update(double distanceCm, boolean reversing) {
        boolean now = seesBall(seen, distanceCm);
        if (now && !seen) count = Math.max(0, count + (reversing ? -1 : 1));
        seen = now;
    }

    static boolean seesBall(boolean wasSeen, double distanceCm) {
        return distanceCm < (wasSeen ? CLEAR_CM : BALL_CM);
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
