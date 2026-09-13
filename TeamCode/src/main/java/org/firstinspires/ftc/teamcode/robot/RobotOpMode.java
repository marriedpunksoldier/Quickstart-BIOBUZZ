package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Shared base for every OpMode (TeleOp now, autos later).
 * <p>
 * Intentionally has NO game-piece subsystems yet. Add subsystem fields here
 * once the BIOBUZZ robot design settles; don't stub them in speculatively.
 * <p>
 * A subclass calls {@link #initSubsystems()} once at the top of its init().
 */
public abstract class RobotOpMode extends OpMode {

    protected final Alliance alliance;

    protected Follower follower;

    protected RobotOpMode(Alliance alliance) {
        this.alliance = alliance;
    }

    /** Enable bulk caching and build the Follower. Call first in init(). */
    protected void initSubsystems() {
        enableBulkCaching(hardwareMap);
        follower = Constants.create(hardwareMap);
        if (follower == null) {
            throw new IllegalStateException(
                    "pedro/Constants.create() returned null - fill it in before driving");
        }
    }

    /**
     * Set every REV hub to AUTO bulk caching: one bulk read per loop instead of
     * a USB round-trip per hardware read, refreshed automatically. Static so a
     * standalone test OpMode can reuse it.
     */
    public static void enableBulkCaching(HardwareMap hardwareMap) {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}
