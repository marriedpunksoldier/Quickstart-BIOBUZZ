package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystems.DriveUtils;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.Locale;

/**
 * TeleOp shared by both alliances; each alliance is a thin subclass.
 * <p>
 * gamepad1: left stick translates (field-centric), right stick X rotates.
 * "Field" forward is the direction the robot faced at INIT, since the start
 * pose is (0, 0, 0) until auto hands its final pose over.
 * <p>
 * gamepad2: right stick X nudges the turret by hand. Otherwise the turret
 * auto-aims at our HIVE's upward CELL whenever the Limelight sees its tags,
 * and holds still when it doesn't.
 * <p>
 * Releasing the drive sticks holds the current pose rather than coasting, which
 * matters in BIOBUZZ: TELEOP has no protected zones and the manual calls the
 * game "highly interactive". Telemetry shows HOLD or MANUAL so the drivers can
 * see which is active.
 */
public abstract class TeleopBase extends RobotOpMode {

    /**
     * Speed below which a stickless robot latches its pose instead of coasting
     * (inches/sec). Pedro's own default is 0.2. Raise it if the robot creeps
     * before grabbing hold; lower it if it latches while still sliding.
     */
    protected static final double HOLD_VELOCITY_THRESHOLD = 0.2;

    /**
     * Power below which {@link ManualDrive#driveOrHold} treats the sticks as
     * released. Zero on purpose: {@link DriveUtils} already deadzones to exactly
     * 0, so any non-zero power here is real driver intent. Pedro's 0.1 default
     * assumes raw sticks and would swallow everything under ~38% deflection
     * once our square curve has been applied.
     */
    private static final double HOLD_INPUT_THRESHOLD = 0.0;

    /** Servo units per loop at full gamepad2 stick: ~1.1 deg of turret, ~55 deg/s at 50 Hz. */
    private static final double MANUAL_TURRET_STEP = 0.01;

    protected LimelightSubsystem limelight;
    protected TurretSubsystem turret;

    protected TeleopBase(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init() {
        initSubsystems();
        limelight = new LimelightSubsystem(hardwareMap, alliance);
        turret = new TurretSubsystem(hardwareMap);
        follower.setPose(startingPose());
        // Nothing moves in INIT: G403 forbids powered movement before TELEOP.
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Limelight", limelight.isConnected()
                ? "OK" : "NOT FOUND (" + limelight.getInitError() + ") - driving still works");
        telemetry.update();
    }

    /** Override for a real start pose. Default is the origin, for practice. */
    protected Pose startingPose() {
        return Pose.zero();
    }

    @Override
    public void start() {
        turret.center();   // first servo command, deliberately not in init()
    }

    @Override
    public void loop() {
        limelight.update();
        aimTurret();

        // driveOrHold latches the current pose when the sticks are released and
        // the robot has stopped, so defenders can't shove us off our spot. It
        // sets Mode.HOLD once, not every loop, and any stick input resumes
        // manual drive.
        ManualDrive.driveOrHold(
                follower, drivePowers(), HOLD_INPUT_THRESHOLD, HOLD_VELOCITY_THRESHOLD);
        follower.update();

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Mode", follower.holding() ? "HOLD" : "MANUAL");
        telemetry.addData("Target CELL tags", limelight.getTargetTags());
        telemetry.addData("tx", String.format(Locale.US, "%.1f deg", limelight.getTxDegrees()));
        telemetry.addData("Turret", String.format(Locale.US, "%.1f deg %s",
                turret.getAngleDeg(), turret.isLockedOn() ? "LOCKED" : ""));
        telemetry.update();
    }

    /** Driver override on gamepad2 right stick X; auto-aim otherwise. */
    private void aimTurret() {
        double manual = DriveUtils.applyDeadzone(gamepad2.right_stick_x);
        if (manual != 0) {
            // Stick right turns the turret the same way as positive tx (right of center).
            turret.setPosition(turret.getPosition() - manual * MANUAL_TURRET_STEP);
        } else {
            turret.aimAtTarget(limelight.getTxDegrees(), limelight.hasTarget());
        }
    }

    private DrivePowers drivePowers() {
        double[] xy = DriveUtils.applyRadialDeadzone(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, DriveUtils.DEFAULT_DEADBAND);
        double forward = DriveUtils.squareCurve(xy[0]);
        double strafe = DriveUtils.squareCurve(xy[1]);
        double turn = DriveUtils.squareCurve(DriveUtils.applyDeadzone(-gamepad1.right_stick_x));
        return ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading());
    }
}
