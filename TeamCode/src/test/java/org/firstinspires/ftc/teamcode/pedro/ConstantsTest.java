package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.algorithm.ForesightConfig;

import org.junit.Test;

/**
 * Covers the half of {@link Constants} that does not touch hardware.
 * <p>
 * {@code drivetrainConfig()} and {@code localizerConfig()} cannot be tested
 * here - they construct Android/FTC SDK types that do not exist on a plain JVM.
 * MecanumTuner and PinpointTuner are what check those.
 * <p>
 * Run: ./gradlew :TeamCode:testDebugUnitTest
 */
public class ConstantsTest {
    private static final double EPS = 1e-9;

    /**
     * The point of this test. Every ConfigVar.required() in ForesightConfig
     * throws if it is read while unset, and the positive()/nonnull() validators
     * throw on a bad value - so merely building the config proves we filled in
     * all twelve required fields with legal values. Unset, that surfaces as a
     * crash in RobotOpMode.initSubsystems() on the robot.
     */
    @Test
    public void foresightConfigSatisfiesEveryRequiredFieldAndValidator() {
        ForesightConfig config = Constants.foresightConfig();

        assertNotNull(config.forwardTranslational.get());
        assertNotNull(config.strafeTranslational.get());
        assertNotNull(config.headingFeedback.get());
        assertNotNull(config.coast.get());
        assertNotNull(config.brake.get());
        assertNotNull(config.linearBrakeCoefficients.get());
        assertNotNull(config.quadraticBrakeCoefficients.get());
        assertNotNull(config.headingBrakeCoefficients.get());

        assertEquals(Constants.MAX_FORWARD_VELOCITY, config.maxAchievableForwardVelocity.get(), EPS);
        assertEquals(Constants.MAX_STRAFE_VELOCITY, config.maxAchievableStrafeVelocity.get(), EPS);
        assertEquals(Constants.NATURAL_FORWARD_DECELERATION, config.naturalForwardDeceleration.get(), EPS);
        assertEquals(Constants.NATURAL_STRAFE_DECELERATION, config.naturalStrafeDeceleration.get(), EPS);
    }

    /**
     * The DERIVED brake coefficients claim to encode "braking is no better than
     * coasting", i.e. Foresight's own coast model v*|v|/(2a). If someone
     * re-tunes deceleration but leaves a stale hand-typed coefficient behind,
     * the robot silently plans stops against the wrong physics.
     */
    @Test
    public void quadraticBrakeCoefficientsMatchTheCoastDownModel() {
        assertEquals(1.0 / (2.0 * Constants.NATURAL_FORWARD_DECELERATION),
                Constants.FORWARD_QUADRATIC_BRAKE, EPS);
        assertEquals(1.0 / (2.0 * Constants.NATURAL_STRAFE_DECELERATION),
                Constants.STRAFE_QUADRATIC_BRAKE, EPS);

        // Diagonal placement matters: [0][0] is forward, [1][1] is strafe.
        assertEquals(Constants.FORWARD_QUADRATIC_BRAKE,
                Constants.foresightConfig().quadraticBrakeCoefficients.get().get(0, 0), EPS);
        assertEquals(Constants.STRAFE_QUADRATIC_BRAKE,
                Constants.foresightConfig().quadraticBrakeCoefficients.get().get(1, 1), EPS);
    }

    /**
     * Guards the port itself: these are last season's measured numbers, and a
     * typo in any of them is invisible until the robot drives badly. Strafe is
     * slower than forward and decelerates harder on a mecanum chassis - if that
     * ordering ever inverts, the two axes have been swapped somewhere.
     */
    @Test
    public void portedValuesKeepTheirMecanumAxisOrdering() {
        assertEquals(60.45640215535802, Constants.MAX_FORWARD_VELOCITY, EPS);
        assertEquals(50.41999492495079, Constants.MAX_STRAFE_VELOCITY, EPS);
        assertEquals(47.47678691176189, Constants.NATURAL_FORWARD_DECELERATION, EPS);
        assertEquals(82.19375016013265, Constants.NATURAL_STRAFE_DECELERATION, EPS);

        assertTrue("strafe should be slower than forward",
                Constants.MAX_STRAFE_VELOCITY < Constants.MAX_FORWARD_VELOCITY);
        assertTrue("strafe should decelerate harder than forward",
                Constants.NATURAL_STRAFE_DECELERATION > Constants.NATURAL_FORWARD_DECELERATION);
    }
}
