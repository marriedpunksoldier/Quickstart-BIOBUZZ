package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * The intake roller (goBILDA 5203 1150 rpm, {@code intake}) with jam reverse
 * ({@link IntakeControl}) and a ball count from a distance sensor at the mouth
 * ({@code intakeSensor}, e.g. a REV Color Sensor V3; {@link BallCounter}).
 * Without the sensor the intake still runs, but nothing stops it at 4.
 * <p>
 * Never run on a robot.
 */
public class Intake {

    /** The motor stands up through miters and turns the other way: flip this if the roller spits balls out. */
    public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;
    /** 1150 rpm × 145.1 counts/rev (goBILDA 5203-2402-0005) / 60. */
    static final double FREE_TICKS_PER_SEC = 1150 * 145.1 / 60.0;

    private final DcMotorEx motor;
    private final DistanceSensor sensor;
    private final IntakeControl control = new IntakeControl();
    private final BallCounter counter;
    private double power = 0;

    public Intake(HardwareMap hardwareMap, int startCount) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setDirection(DIRECTION);
        // Plain power; the encoder still reports velocity for jam detection.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sensor = hardwareMap.tryGet(DistanceSensor.class, "intakeSensor");
        counter = new BallCounter(startCount);
    }

    /** Call every loop. {@code wantOut} wins over {@code wantIn}. */
    public void update(double now, boolean wantIn, boolean wantOut) {
        if (sensor != null) counter.update(sensor.getDistance(DistanceUnit.CM), power < 0);
        boolean full = sensor != null && counter.full();
        power = control.update(now, wantIn, wantOut, full, motor.getVelocity() / FREE_TICKS_PER_SEC);
        motor.setPower(power);
    }

    public int count() {
        return counter.count();
    }

    /** After a volley, or when the drivers correct a miscount. */
    public void setCount(int n) {
        counter.set(n);
    }

    public boolean hasSensor() {
        return sensor != null;
    }

    public IntakeControl.State state() {
        return control.state();
    }
}
