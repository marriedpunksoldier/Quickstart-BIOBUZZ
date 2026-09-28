package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Follower wiring: mecanum drivetrain + goBILDA Pinpoint odometry + Foresight.
 * <p>
 * <b>Every number here is a PLACEHOLDER carried over from the 2025-2026 DECODE
 * chassis, or derived from those numbers. None of it has been measured on the
 * BIOBUZZ robot, which is not built yet.</b> The wiring (which classes, which
 * order, which units) is correct for Pedro 3.0.1 and is the part worth keeping;
 * the values exist so the robot can be driven far enough to run the tuners.
 * <p>
 * Each constant is tagged with where it came from:
 * <ul>
 *   <li><b>PORTED</b> - a tuned value from last season's Constants. Right
 *       order of magnitude if the chassis really is similar, still wrong in
 *       detail (weight distribution, battery wear and wheel wear all shift
 *       these).</li>
 *   <li><b>DERIVED</b> - computed from a PORTED value because Pedro 3.0
 *       requires an input that 2.x had no equivalent for. Physics estimate,
 *       never measured.</li>
 *   <li><b>UNKNOWN</b> - no data exists. Deliberately neutral.</li>
 * </ul>
 * <p>
 * Replace all of it by running {@code pedro/procedures/PinpointTuner},
 * {@code MecanumTuner} and {@code ForesightTuner} once the robot exists; each
 * one prints a ready-to-paste config block matching the shapes below.
 * <p>
 * Configs are methods, not static fields, so that the unit test can exercise
 * {@link #foresightConfig()} on a plain JVM without loading the Android-only
 * hardware classes the other two touch.
 */
public class Constants {

    // ---- PORTED from Archive/Quickstart 2025 .../pedroPathing/Constants.java ----

    /** PORTED: MecanumConstants.xVelocity. Max forward speed, in/s. */
    static final double MAX_FORWARD_VELOCITY = 60.45640215535802;

    /** PORTED: MecanumConstants.yVelocity. Max strafe speed, in/s. */
    static final double MAX_STRAFE_VELOCITY = 50.41999492495079;

    /** PORTED: |forwardZeroPowerAcceleration|. Coast-down rate, in/s^2. */
    static final double NATURAL_FORWARD_DECELERATION = 47.47678691176189;

    /** PORTED: |lateralZeroPowerAcceleration|. Coast-down rate, in/s^2. */
    static final double NATURAL_STRAFE_DECELERATION = 82.19375016013265;

    /** PORTED: translationalPIDFCoefficients kP. */
    static final double TRANSLATIONAL_KP = 0.3;

    /** PORTED: headingPIDFCoefficients kP. */
    static final double HEADING_KP = 0.9;

    // ---- DERIVED: inputs Pedro 3.0 requires that 2.x had no equivalent for ----

    /**
     * DERIVED: kV for the coast and brake feedforwards, in power per in/s.
     * 1/maxVelocity is the definition of kV at full stick: commanding the
     * robot's top speed asks for power 1.0. ForesightTuner measures these two
     * separately ("coast kV" / "brake kV"); we have one number for both.
     */
    static final double FEEDFORWARD_KV = 1.0 / MAX_FORWARD_VELOCITY;

    /**
     * DERIVED: quadratic braking coefficients, in seconds^2 per inch.
     * <p>
     * Foresight models stopping distance as
     * {@code quadratic*v*|v| + linear*v} ({@code Foresight.getBrakeDisplacement}),
     * and models a pure coast as {@code v*|v|/(2a)}
     * ({@code getCoastDisplacement}). Setting quadratic to 1/(2a) with a zero
     * linear term therefore says "assume braking is no better than coasting."
     * <p>
     * That is deliberately pessimistic: real braking stops shorter, so the
     * robot begins braking early and undershoots rather than overshooting.
     * Undershoot is corrected by the translational controller; overshoot into a
     * field element is not. ForesightTuner replaces both with a real fit.
     */
    static final double FORWARD_QUADRATIC_BRAKE = 1.0 / (2.0 * NATURAL_FORWARD_DECELERATION);

    /** DERIVED: see {@link #FORWARD_QUADRATIC_BRAKE}. */
    static final double STRAFE_QUADRATIC_BRAKE = 1.0 / (2.0 * NATURAL_STRAFE_DECELERATION);

    /** DERIVED: the pure-coast model has no linear term. */
    static final double LINEAR_BRAKE = 0.0;

    /**
     * UNKNOWN: rotational braking, (linear, quadratic) on omega. Last season's
     * Constants had no angular deceleration of any kind to derive these from,
     * so they predict no rotational overshoot at all. The robot will brake late
     * on heading and lean on HEADING_KP feedback to settle. Acceptable only
     * because this config is not meant to run a match. ForesightTuner's
     * HeadingBraking step measures both.
     */
    static final Vector2D HEADING_BRAKE = Vector2D.cartesian(0.0, 0.0);

    /**
     * Foresight tuning. Safe to touch from a JVM unit test: everything here is
     * from com.pedropathing:core, which is a plain jar.
     */
    public static ForesightConfig foresightConfig() {
        return new ForesightConfig(c -> {
            c.forwardTranslational.set(Controller.proportional(TRANSLATIONAL_KP));
            c.strafeTranslational.set(Controller.proportional(TRANSLATIONAL_KP));
            c.headingFeedback.set(Controller.proportional(HEADING_KP));

            c.coast.set(Controller.proportionalFeedforward(FEEDFORWARD_KV));
            c.brake.set(Controller.proportionalFeedforward(FEEDFORWARD_KV));

            c.linearBrakeCoefficients.set(Matrix.diag(LINEAR_BRAKE, LINEAR_BRAKE));
            c.quadraticBrakeCoefficients.set(
                    Matrix.diag(FORWARD_QUADRATIC_BRAKE, STRAFE_QUADRATIC_BRAKE));
            c.headingBrakeCoefficients.set(HEADING_BRAKE);

            c.maxAchievableForwardVelocity.set(MAX_FORWARD_VELOCITY);
            c.maxAchievableStrafeVelocity.set(MAX_STRAFE_VELOCITY);
            c.naturalForwardDeceleration.set(NATURAL_FORWARD_DECELERATION);
            c.naturalStrafeDeceleration.set(NATURAL_STRAFE_DECELERATION);
        });
    }

    /**
     * Motor names and directions, PORTED verbatim. These are wiring facts, not
     * tuned physics, so they survive the season change as long as the hardware
     * map keeps the same four names and the motors are mounted the same way.
     * Confirm with MecanumTuner before trusting the directions.
     */
    public static MecanumConfig drivetrainConfig() {
        return new MecanumConfig(c -> {
            c.frontLeftName.set("leftFront");
            c.backLeftName.set("leftRear");
            c.frontRightName.set("rightFront");
            c.backRightName.set("rightRear");
            c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        });
    }

    /**
     * Pinpoint odometry, PORTED. 2.x named the offsets after the pod they
     * describe; 3.0 names them after the axis each pod measures, so
     * {@code forwardPodY -> xPodOffset} and {@code strafePodX -> yPodOffset}
     * (PinpointLocalizer passes them straight to
     * {@code GoBildaPinpointDriver.setOffsets(x, y, unit)}).
     * <p>
     * The offsets are pod positions on last season's chassis and are wrong the
     * moment anything is remounted. PinpointTuner measures them by spinning the
     * robot 180 degrees.
     */
    public static PinpointConfig localizerConfig() {
        return new PinpointConfig(c -> {
            c.name.set("pinpoint");
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            c.xPodOffset.set(1.5);   // PORTED: forwardPodY
            c.yPodOffset.set(-4.0);  // PORTED: strafePodX
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
        });
    }

    /** Localizer first, then drivetrain, then algorithm - see Follower.java:40. */
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig()),
                new Mecanum(h, drivetrainConfig()),
                new Foresight(foresightConfig()));
    }

    private Constants() {}
}
