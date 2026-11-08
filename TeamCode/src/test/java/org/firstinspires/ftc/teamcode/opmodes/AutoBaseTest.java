package org.firstinspires.ftc.teamcode.opmodes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.junit.Test;

/** Rule checks on the AUTO poses; they say nothing about whether the robot gets there. */
public class AutoBaseTest {

    private static final double W = Alliance.FIELD_SIZE;
    private static final double HALF = AutoBase.HALF_ROBOT;
    private static final Pose[] POSES = {
            AutoBase.START, AutoBase.SHOOT, AutoBase.FLOWER_APPROACH, AutoBase.FLOWER, AutoBase.PARK};

    @Test
    public void everyPoseKeepsTheWholeRobotOnOurSideAndInTheField() {
        for (Alliance a : Alliance.values()) {
            for (Pose blue : POSES) {
                Pose p = a.pose(blue);
                String msg = a + " " + p;
                // G402: Blue is columns D-F (x > W/2), Red A-C (x < W/2).
                if (a == Alliance.BLUE) assertTrue(msg, p.x() - HALF >= W / 2);
                else assertTrue(msg, p.x() + HALF <= W / 2);
                assertTrue(msg, p.x() - HALF >= 0 && p.x() + HALF <= W);
                assertTrue(msg, p.y() - HALF >= 0 && p.y() + HALF <= W);
            }
        }
    }

    @Test
    public void startTouchesTheRearWall() {
        assertEquals(W, AutoBase.START.y() + HALF, 1e-9);  // G304: touching the perimeter wall
    }

    @Test
    public void parkOverlapsTheLoadingZoneWithoutTouchingTheWall() {
        double rightEdge = AutoBase.PARK.x() + HALF;
        assertTrue(rightEdge < W);        // Q21: on the wall at 0:00 loses LEAVE
        assertTrue(rightEdge > W - 11);   // zone is 11 in deep from the wall
        assertTrue(AutoBase.PARK.y() > 23.6 && AutoBase.PARK.y() < 47.2);  // tile F2
    }

    @Test
    public void shootPoseFacesTheCellFromInsideTheShotTable() {
        Pose s = AutoBase.SHOOT;
        double dx = AutoBase.CELL_X - s.x();
        double dy = AutoBase.CELL_Y - s.y();
        double range = Math.hypot(dx, dy);
        assertTrue("range " + range, range >= 36 && range <= 92);
        Pose oneInchAhead = new Pose(s.x() + Math.cos(s.heading()), s.y() + Math.sin(s.heading()), 0);
        assertEquals(range - 1, Math.hypot(AutoBase.CELL_X - oneInchAhead.x(), AutoBase.CELL_Y - oneInchAhead.y()), 1e-6);
    }
}
