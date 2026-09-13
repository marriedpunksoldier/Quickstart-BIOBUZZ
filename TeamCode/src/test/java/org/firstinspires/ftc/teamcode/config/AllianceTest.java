package org.firstinspires.ftc.teamcode.config;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;

import com.pedropathing.math.Pose;

import org.junit.Test;

/** Run: ./gradlew :TeamCode:testDebugUnitTest */
public class AllianceTest {
    private static final double EPS = 1e-9;

    private static void assertPose(double x, double y, double headingRad, Pose actual) {
        assertEquals("x", x, actual.x(), EPS);
        assertEquals("y", y, actual.y(), EPS);
        assertEquals("heading", headingRad, actual.heading(), EPS);
    }

    @Test
    public void blueIsUnchanged() {
        Pose blue = new Pose(10, 20, 1);
        assertSame(blue, Alliance.BLUE.pose(blue));
    }

    @Test
    public void redIsRotated180AboutFieldCenter() {
        Pose red = Alliance.RED.pose(new Pose(10, 20, 0));
        assertPose(131.5, 121.5, Math.PI, red);
    }

    @Test
    public void redHeadingWrapsIntoRange() {
        // 270 deg + 180 deg = 450 deg -> 90 deg
        assertEquals(Math.PI / 2, Alliance.RED.heading(Math.toRadians(270)), EPS);
    }

    @Test
    public void rotatingTwiceIsIdentity() {
        Pose blue = new Pose(33, 100, Math.toRadians(30));
        Pose back = Alliance.RED.pose(Alliance.RED.pose(blue));
        assertPose(blue.x(), blue.y(), blue.heading(), back);
    }

    @Test
    public void fieldCenterIsFixedPoint() {
        double c = Alliance.FIELD_SIZE / 2;
        Pose red = Alliance.RED.pose(new Pose(c, c, 0));
        assertPose(c, c, Math.PI, red);
    }

    @Test
    public void poseFactoryMatchesPose() {
        Pose viaFactory = Alliance.RED.poses().of(10, 20, 90);
        assertPose(131.5, 121.5, Math.toRadians(270), viaFactory);
    }
}
