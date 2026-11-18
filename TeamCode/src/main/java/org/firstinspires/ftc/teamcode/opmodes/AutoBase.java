package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystems.BallCounter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.Locale;

/**
 * AUTO shared by both alliances: fire the 4 pre-loads into the upward CELL (it
 * starts with 3 NECTAR, so this should TIP it), pull the 4 POLLEN out of a
 * FLOWER with the wedge, fire them at the new upward CELL, then PARK.
 * <p>
 * Poses are written once in BLUE (Pedro Visualizer frame: inches, 141.5 in
 * field, x left to right and y away from the audience) and rotated for RED by
 * {@link Alliance#pose(Pose)}. <b>Every pose and time here is a guess until the
 * robot runs</b>; the ones marked MEASURE need the real field.
 * <p>
 * The feed and wedge have no hardware yet: {@link #setFeed} and
 * {@link #setWedge} are empty until they're wired.
 * Never run on a robot.
 */
public abstract class AutoBase extends RobotOpMode {

    /** Half the 18 in robot, for keeping poses on our side and off walls. */
    static final double HALF_ROBOT = 9.0;

    /** Blue's starting upward CELL (the rear one, tags 42-45), centre in plan. */
    static final double CELL_X = Alliance.FIELD_SIZE / 2 + 12.75;  // HIVE centres 25.5 in apart
    static final double CELL_Y = Alliance.FIELD_SIZE / 2 + 9.42;   // CELLs 18.84 in apart

    /** Against the rear wall on tile D6, facing the audience. MEASURE: keep clear of the rear FLOWER (G304.D). */
    static final Pose START = blue(84, Alliance.FIELD_SIZE - HALF_ROBOT, 270);
    /** ~38 in from the CELL, inside the shot table's 36-92 in. */
    static final Pose SHOOT = aimedAt(100, 114);
    /** MEASURE: the Blue FLOWER on the right wall; wedge stowed, about 8 in short of it. */
    static final Pose FLOWER_APPROACH = blue(119, 94.3, 0);
    /** MEASURE: robot pose with the wedge tongue under the FLOWER's bottom ball. */
    static final Pose FLOWER = blue(127, 94.3, 0);
    /** Tile F2 LOADING ZONE: 6 in into the 11 in zone, 5 in off the wall (Q21: touching the wall loses LEAVE). */
    static final Pose PARK = blue(Alliance.FIELD_SIZE - 5 - HALF_ROBOT, 35.4, 90);

    /** Placeholder until the shot table is checked on the bench. */
    static final double SHOT_RPM = 3000;
    /** Time-based until the column ball counter exists. */
    static final double FIRE_SEC = 2.0;
    static final double WEDGE_DROP_SEC = 0.5;
    static final double PULL_SEC = 1.5;
    /** Fire anyway if the turret hasn't locked on by then (e.g. no Limelight). */
    static final double AIM_TIMEOUT_SEC = 1.0;
    /** Give up whatever we're doing and PARK; AUTO is 30 s. */
    static final double PARK_BY_SEC = 25.0;

    enum Step { DRIVE_TO_SHOOT, AIM, FIRE, DRIVE_TO_FLOWER, DROP_WEDGE, PULL_FLOWER, PARK, DONE }

    private Shooter shooter;
    private Intake intake;
    private boolean wantIntake = false;
    private TurretSubsystem turret;
    private LimelightSubsystem limelight;

    private Step step;
    private double stepStart;
    private int volleys = 0;

    protected AutoBase(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init() {
        initSubsystems();
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap, BallCounter.LIMIT);   // starts with 4 pre-loads
        turret = new TurretSubsystem(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap, alliance);
        follower.setPose(alliance.pose(START));
        // Nothing moves in INIT (G304: motionless after INIT).
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Limelight", limelight.isConnected()
                ? "OK" : "NOT FOUND (" + limelight.getInitError() + ") - will fire unaimed");
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
        turret.center();
        shooter.setRpm(SHOT_RPM);
        driveTo(SHOOT, Step.DRIVE_TO_SHOOT);
    }

    @Override
    public void loop() {
        limelight.update();
        turret.aimAtTarget(limelight.getTxDegrees(), limelight.hasTarget());
        if (getRuntime() > PARK_BY_SEC && step.ordinal() < Step.PARK.ordinal()) park();
        runStep(getRuntime() - stepStart);
        intake.update(getRuntime(), wantIntake, false);
        follower.update();

        telemetry.addData("Step", step);
        telemetry.addData("Volleys", volleys);
        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Flywheel", String.format(Locale.US, "%.0f / %.0f rpm %s",
                shooter.getRpm(), SHOT_RPM, shooter.atSpeed() ? "READY" : ""));
        telemetry.addData("Turret", turret.isLockedOn() ? "LOCKED" : "searching");
        telemetry.addData("Intake", intake.state() + (intake.hasSensor() ? " balls " + intake.count() : " (no sensor)"));
        telemetry.update();
    }

    private void runStep(double t) {
        boolean arrived = !follower.following();
        switch (step) {
            case DRIVE_TO_SHOOT:
                if (arrived) enter(Step.AIM);
                break;
            case AIM:
                if (shooter.atSpeed() && (turret.isLockedOn() || t > AIM_TIMEOUT_SEC)) {
                    setFeed(true);
                    enter(Step.FIRE);
                }
                break;
            case FIRE:
                if (t < FIRE_SEC) break;
                setFeed(false);
                intake.setCount(0);   // time-based volley: assume it emptied
                volleys++;
                // The robot now holds 0, so the wedge may tip a full stack in (G407, TU05).
                if (volleys == 1) driveTo(FLOWER_APPROACH, Step.DRIVE_TO_FLOWER);
                else park();
                break;
            case DRIVE_TO_FLOWER:
                if (arrived) {
                    setWedge(true);
                    setIntake(true);
                    enter(Step.DROP_WEDGE);
                }
                break;
            case DROP_WEDGE:
                if (t > WEDGE_DROP_SEC) driveTo(FLOWER, Step.PULL_FLOWER);
                break;
            case PULL_FLOWER:
                if (arrived && t > PULL_SEC) {
                    setWedge(false);
                    setIntake(false);
                    // The wedge tips 4 in touching, which the mouth sensor can't split.
                    intake.setCount(BallCounter.LIMIT);
                    driveTo(SHOOT, Step.DRIVE_TO_SHOOT);
                }
                break;
            case PARK:
                if (arrived) enter(Step.DONE);
                break;
            case DONE:
                break;
        }
    }

    private void park() {
        setFeed(false);
        setIntake(false);
        setWedge(false);
        shooter.setRpm(0);
        driveTo(PARK, Step.PARK);
    }

    private void driveTo(Pose bluePose, Step next) {
        follower.follow(Paths.line(follower.pose(), alliance.pose(bluePose)));
        enter(next);
    }

    private void enter(Step next) {
        step = next;
        stepStart = getRuntime();
    }

    /** TODO: run the column drive into the flywheel. No hardware name yet. */
    protected void setFeed(boolean on) { }

    /** Intake roller; stops itself at 4 balls and reverses out of jams ({@link Intake}). */
    protected void setIntake(boolean on) {
        wantIntake = on;
    }

    /** TODO: FLOWER wedge servo, down = true. No hardware name yet. */
    protected void setWedge(boolean down) { }

    static Pose blue(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    /** A pose at (x, y) with the robot's front pointing at the starting CELL. */
    static Pose aimedAt(double x, double y) {
        return new Pose(x, y, Math.atan2(CELL_Y - y, CELL_X - x));
    }
}
