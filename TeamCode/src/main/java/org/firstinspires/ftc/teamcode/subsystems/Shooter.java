package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The flywheel: two bare goBILDA 5000-series motors, one on each end of the
 * shaft through 1:1 miters, so the left encoder reads the flywheel directly.
 * Same wiring as {@code ShooterBench}; if the bench test shows one motor is
 * enough, drop {@code shooterRight}.
 * <p>
 * Never run on a robot.
 */
public class Shooter {

    /** goBILDA spec: 28 counts per rev at the output shaft (no gearbox). */
    public static final double COUNTS_PER_REV = 28.0;

    /** The motors face each other across the shaft, so one turns the other way. */
    public static final DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;

    /** "At speed" means within 2% of target, the bench sheet's recovery line. */
    static final double READY_FRACTION = 0.98;

    private final DcMotorEx left;
    private final DcMotorEx right;
    private double targetRpm = 0;

    public Shooter(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        right = hardwareMap.get(DcMotorEx.class, "shooterRight");
        for (DcMotorEx m : new DcMotorEx[] {left, right}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Coast down: braking a flywheel shocks the miters.
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        left.setDirection(LEFT_DIRECTION);
        right.setDirection(RIGHT_DIRECTION);
    }

    /** Spin to {@code rpm}, or coast to a stop for 0. */
    public void setRpm(double rpm) {
        targetRpm = Math.max(0, rpm);
        if (targetRpm == 0) {
            left.setPower(0);
            right.setPower(0);
            return;
        }
        double ticksPerSec = targetRpm * COUNTS_PER_REV / 60.0;
        left.setVelocity(ticksPerSec);
        right.setVelocity(ticksPerSec);
    }

    public double getRpm() {
        return left.getVelocity() * 60.0 / COUNTS_PER_REV;
    }

    public boolean atSpeed() {
        return isAtSpeed(getRpm(), targetRpm);
    }

    static boolean isAtSpeed(double rpm, double target) {
        return target > 0 && rpm >= READY_FRACTION * target;
    }
}
