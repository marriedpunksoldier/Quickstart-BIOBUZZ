package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.junit.Test;

public class LimelightSubsystemTest {

    private static final int RED = LimelightSubsystem.firstHiveTag(Alliance.RED);
    private static final int BLUE = LimelightSubsystem.firstHiveTag(Alliance.BLUE);

    @Test
    public void hiveTagRangesMatchManual() {
        assertEquals(30, RED);
        assertEquals(38, BLUE);
    }

    @Test
    public void aimsAtTheHigherCellAndAveragesItsTx() {
        // Red rear CELL (30-33) low in frame, audience CELL (34-37) high: pick 34-37.
        int[] ids = {30, 31, 34, 35};
        double[] tx = {-10, -8, 2, 4};
        double[] ty = {5, 5, 20, 22};
        assertArrayEquals(new double[] {1, 3.0}, LimelightSubsystem.aim(ids, tx, ty, RED), 1e-9);
    }

    @Test
    public void ignoresTheOpponentHive() {
        // Blue tags 38-45 are higher in frame but aren't ours when we're Red.
        int[] ids = {38, 42, 33};
        double[] tx = {0, 0, -6};
        double[] ty = {40, 40, 10};
        assertArrayEquals(new double[] {0, -6}, LimelightSubsystem.aim(ids, tx, ty, RED), 1e-9);
    }

    @Test
    public void idJustBelowOurRangeIsNotCountedAsCellZero() {
        // 29 - 30 = -1, and -1 / 4 truncates to 0 in Java; it must still be rejected.
        assertNull(LimelightSubsystem.aim(new int[] {29}, new double[] {1}, new double[] {1}, RED));
    }

    @Test
    public void noOwnTagsMeansNoTarget() {
        assertNull(LimelightSubsystem.aim(new int[] {}, new double[] {}, new double[] {}, BLUE));
        assertNull(LimelightSubsystem.aim(new int[] {37}, new double[] {1}, new double[] {1}, BLUE));
    }
}
