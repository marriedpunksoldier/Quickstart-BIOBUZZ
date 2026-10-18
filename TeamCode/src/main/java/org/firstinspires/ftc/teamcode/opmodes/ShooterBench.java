package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Bench test for the flywheel: one motor vs two (Shooter Bench Test Sheet).
 * Not a competition OpMode; it needs no drivetrain, so it runs off the robot.
 * <p>
 * Two bare 5000-0002-4008 motors drive the flywheel shaft from both ends
 * through 1:1 2319 miters, so either encoder reads the flywheel directly.
 * For the one-motor run, unplug shooterRight's power cable only; its encoder
 * stays plugged, and shooterLeft keeps the same target.
 * <p>
 * gamepad1: A = spin on/off, B = reset burst stats, dpad up/down = target RPM.
 */
@TeleOp(name = "Shooter Bench", group = "Test")
public class ShooterBench extends OpMode {

    /** goBILDA spec: 28 counts per rev at the output shaft (no gearbox). */
    static final double COUNTS_PER_REV = 28.0;
    /** goBILDA's tested free speed at 12 V. */
    static final double MAX_RPM = 5800.0;
    static final double DEFAULT_RPM = 3000.0;
    static final double RPM_STEP = 100.0;

    /** "Within 2% of target" on the bench sheet. */
    static final double RECOVERED_FRACTION = 0.98;

    /**
     * The motors face each other across the shaft, so one must turn the other
     * way. If telemetry shows a direction fault, flip this. If balls come out
     * backwards, flip LEFT_DIRECTION and this together.
     */
    static final DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;

    private DcMotorEx left;
    private DcMotorEx right;
    private VoltageSensor battery;
    private final ElapsedTime clock = new ElapsedTime();
    private final RpmStats stats = new RpmStats();

    private double targetRpm = DEFAULT_RPM;
    private boolean spinning = false;
    private String fault = null;
    private boolean lastA, lastB, lastUp, lastDown;

    @Override
    public void init() {
        left = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        right = hardwareMap.get(DcMotorEx.class, "shooterRight");
        battery = hardwareMap.voltageSensor.iterator().next();
        for (DcMotorEx m : new DcMotorEx[] {left, right}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Coast down: braking a flywheel shocks the miters.
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        left.setDirection(LEFT_DIRECTION);
        right.setDirection(RIGHT_DIRECTION);
        telemetry.addLine("A spin, B reset burst, dpad up/down target RPM");
    }

    @Override
    public void loop() {
        double t = clock.seconds();
        handleButtons(t);

        double leftRpm = toRpm(left.getVelocity());
        double rightRpm = toRpm(right.getVelocity());
        if (spinning) {
            stats.update(t, leftRpm, targetRpm);
            fault = RpmStats.fault(leftRpm, rightRpm, targetRpm, t - stats.startTime);
            if (fault != null) spinning = false;
        }

        if (spinning) {
            double ticksPerSec = targetRpm * COUNTS_PER_REV / 60.0;
            left.setVelocity(ticksPerSec);
            right.setVelocity(ticksPerSec);
        } else {
            left.setPower(0);
            right.setPower(0);
        }

        telemetry.addData("Spinning", spinning);
        if (fault != null) telemetry.addData("FAULT", fault);
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Flywheel RPM (left enc)", "%.0f", leftRpm);
        telemetry.addData("Right enc RPM", "%.0f", rightRpm);
        telemetry.addData("Spin-up (s)", fmt(stats.spinUpSec));
        telemetry.addData("Burst min RPM", stats.minRpm == Double.POSITIVE_INFINITY ? "-" : String.format("%.0f", stats.minRpm));
        telemetry.addData("Recovery (s)", fmt(stats.recoverySec));
        telemetry.addData("Current L / R (A)", "%.1f / %.1f",
                left.getCurrent(CurrentUnit.AMPS), right.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Battery (V)", "%.2f", battery.getVoltage());
        telemetry.update();
    }

    @Override
    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }

    private void handleButtons(double t) {
        if (gamepad1.a && !lastA) {
            spinning = !spinning;
            fault = null;
            if (spinning) stats.start(t);
        }
        if (gamepad1.b && !lastB) stats.resetBurst();
        if (gamepad1.dpad_up && !lastUp) changeTarget(RPM_STEP, t);
        if (gamepad1.dpad_down && !lastDown) changeTarget(-RPM_STEP, t);
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
    }

    private void changeTarget(double delta, double t) {
        targetRpm = Math.max(RPM_STEP, Math.min(MAX_RPM, targetRpm + delta));
        if (spinning) stats.start(t); // re-time the spin-up to the new target
    }

    static double toRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / COUNTS_PER_REV;
    }

    private static String fmt(double sec) {
        return Double.isNaN(sec) ? "-" : String.format("%.2f", sec);
    }

    /**
     * Spin-up, burst minimum and recovery, from RPM samples. Plain Java so it
     * runs in a JVM unit test.
     * <p>
     * Recovery is timed from the lowest RPM to back within 2% of target. In a
     * burst the lowest point comes at or just after the last ball, so this
     * stands in for the sheet's "last ball to recovered".
     */
    static final class RpmStats {
        /** Shaft below this fraction of target after STALL_CHECK_SEC: stop. */
        static final double STALL_FRACTION = 0.2;
        static final double STALL_CHECK_SEC = 1.5;
        /** Below this, encoder noise could flip the sign. */
        static final double DIRECTION_CHECK_RPM = 200.0;

        double startTime;
        double spinUpSec = Double.NaN;
        double minRpm = Double.POSITIVE_INFINITY;
        double minTime;
        double recoverySec = Double.NaN;

        void start(double t) {
            startTime = t;
            spinUpSec = Double.NaN;
            resetBurst();
        }

        void resetBurst() {
            minRpm = Double.POSITIVE_INFINITY;
            recoverySec = Double.NaN;
        }

        void update(double t, double rpm, double target) {
            boolean recovered = rpm >= RECOVERED_FRACTION * target;
            if (Double.isNaN(spinUpSec)) {
                if (recovered) spinUpSec = t - startTime;
                return; // the climb to speed isn't a burst dip
            }
            if (rpm < minRpm) {
                minRpm = rpm;
                minTime = t;
                recoverySec = Double.NaN;
            } else if (recovered && Double.isNaN(recoverySec)
                    && minRpm < RECOVERED_FRACTION * target) {
                recoverySec = t - minTime;
            }
        }

        /** Why to stop spinning, or null if all looks right. */
        static String fault(double leftRpm, double rightRpm, double target, double secSinceStart) {
            if (Math.abs(leftRpm) > DIRECTION_CHECK_RPM && Math.abs(rightRpm) > DIRECTION_CHECK_RPM
                    && Math.signum(leftRpm) != Math.signum(rightRpm)) {
                return "encoders disagree: flip RIGHT_DIRECTION";
            }
            if (secSinceStart > STALL_CHECK_SEC && Math.abs(leftRpm) < STALL_FRACTION * target) {
                return "shaft not spinning up: motors fighting or jammed";
            }
            return null;
        }
    }
}
