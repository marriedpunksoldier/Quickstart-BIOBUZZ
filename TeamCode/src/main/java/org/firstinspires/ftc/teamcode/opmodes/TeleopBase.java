package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystems.DriveUtils;

/**
 * Drive-only TeleOp shared by both alliances; each alliance is a thin
 * subclass. Add mechanism controls here once BIOBUZZ subsystems exist.
 * <p>
 * gamepad1: left stick translates (field-centric), right stick X rotates.
 * "Field" forward is the direction the robot faced at INIT, since the start
 * pose is (0, 0, 0) until auto hands its final pose over.
 */
public abstract class TeleopBase extends RobotOpMode {

    protected TeleopBase(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init() {
        initSubsystems();
        follower.setPose(startingPose());
        // No drive commands in INIT: G403 forbids powered movement before TELEOP.
        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }

    /** Override for a real start pose. Default is the origin, for practice. */
    protected Pose startingPose() {
        return Pose.zero();
    }

    @Override
    public void loop() {
        follower.manual(drivePowers());
        follower.update();

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pose", follower.pose());
        telemetry.update();
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
