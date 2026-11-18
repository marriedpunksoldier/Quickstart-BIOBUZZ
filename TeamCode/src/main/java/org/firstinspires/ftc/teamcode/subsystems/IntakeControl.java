package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Intake roller logic with no SDK types, so it runs on a plain JVM.
 * <ul>
 *   <li><b>Stop when full:</b> holding intake with 4 balls aboard gives 0 power (TU05, 5+ is a penalty).</li>
 *   <li><b>Jam reverse:</b> if the roller runs below {@link #JAM_FRACTION} of free speed for
 *   {@link #JAM_SEC}, reverse for {@link #UNJAM_SEC}, then try again. After {@link #MAX_UNJAMS}
 *   tries in a row it stops ({@link State#JAMMED}) until the driver lets go of intake.</li>
 * </ul>
 * ⚠ The conveyor is belted to the intake shaft, so a reverse also backs the conveyor up.
 * Every number here is a guess until the intake runs on real balls.
 */
public class IntakeControl {

    public enum State { IDLE, IN, OUT, FULL, UNJAM, JAMMED }

    /** Below this share of free speed counts as stalled. */
    static final double JAM_FRACTION = 0.25;
    /** Stalled this long = jam. Grabbing a ball dips the roller briefly; this rides that out. */
    static final double JAM_SEC = 0.25;
    static final double UNJAM_SEC = 0.3;
    static final int MAX_UNJAMS = 3;
    /** Running free this long forgets earlier jams. */
    static final double CLEAR_SEC = 1.0;

    static final double IN_POWER = 1.0;
    static final double OUT_POWER = -1.0;
    static final double UNJAM_POWER = -0.6;

    private State state = State.IDLE;
    private double stallSince = -1;
    private double freeSince = -1;
    private double unjamUntil = 0;
    private int unjams = 0;

    /**
     * @param now           seconds, any monotonic clock
     * @param speedFraction roller speed / free speed, positive = intaking
     * @return motor power, positive = intaking
     */
    public double update(double now, boolean wantIn, boolean wantOut, boolean full, double speedFraction) {
        if (wantOut) return idle(State.OUT, OUT_POWER);
        if (!wantIn) return idle(State.IDLE, 0);
        if (state == State.JAMMED) return 0;
        if (state == State.UNJAM) {
            if (now < unjamUntil) return UNJAM_POWER;
            stallSince = -1;   // give it JAM_SEC to spin back up
        }
        if (full) {
            state = State.FULL;
            stallSince = -1;
            return 0;
        }
        state = State.IN;
        if (speedFraction >= JAM_FRACTION) {
            stallSince = -1;
            if (freeSince < 0) freeSince = now;
            if (now - freeSince >= CLEAR_SEC) unjams = 0;
            return IN_POWER;
        }
        freeSince = -1;
        if (stallSince < 0) stallSince = now;
        if (now - stallSince < JAM_SEC) return IN_POWER;
        if (++unjams > MAX_UNJAMS) {
            state = State.JAMMED;
            return 0;
        }
        state = State.UNJAM;
        unjamUntil = now + UNJAM_SEC;
        return UNJAM_POWER;
    }

    private double idle(State s, double power) {
        state = s;
        stallSince = -1;
        freeSince = -1;
        unjams = 0;
        return power;
    }

    public State state() {
        return state;
    }
}
