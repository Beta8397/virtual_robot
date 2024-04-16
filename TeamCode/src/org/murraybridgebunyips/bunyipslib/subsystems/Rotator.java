package org.murraybridgebunyips.bunyipslib.subsystems;

import static org.murraybridgebunyips.bunyipslib.Text.round;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Degrees;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.PivotMotor;
import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.tasks.ContinuousTask;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.NoTimeoutTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.function.DoubleSupplier;

/**
 * Controls a generic holdable arm as a rotator with limited degrees positions.
 *
 * @author Lachlan Paul, 2024
 * @author Lucas Bubner, 2024
 * @see HoldableActuator
 */
public class Rotator extends BunyipsSubsystem {
    // Encoder lower limit angle
    private Measure<Angle> MIN_LIMIT = Degrees.of(Double.MIN_VALUE);

    // Encoder upper limit angle
    private Measure<Angle> MAX_LIMIT = Degrees.of(Double.MAX_VALUE);

    // Lower power clamp
    private double LOWER_POWER = -1.0;

    // Upper power clamp
    private double UPPER_POWER = 1.0;

    // Tolerance for the rotator in degrees, default 5
    private Measure<Angle> TOLERANCE = Degrees.of(5);

    // Name of the rotator for telemetry
    private String NAME = "Rotator";

    private PivotMotor pivot;
    private double power;
    private boolean lockout;

    /**
     * Create a new ClawRotator
     *
     * @param motor              the motor to use as the rotator
     * @param ticksPerRevolution the number of ticks per revolution of the motor
     */
    public Rotator(DcMotorEx motor, double ticksPerRevolution) {
        if (!assertParamsNotNull(motor)) return;
        pivot = new PivotMotor(motor, ticksPerRevolution);
        pivot.reset();
        pivot.holdCurrentPosition();
    }

    /**
     * Set the name of the rotator for telemetry.
     *
     * @param newName the name to set
     * @return this
     */
    public Rotator withName(String newName) {
        NAME = newName;
        return this;
    }

    /**
     * Set the tolerance for the target angle of the rotator.
     *
     * @param tolerance the tolerance to set
     * @return this
     */
    public Rotator withTolerance(Measure<Angle> tolerance) {
        TOLERANCE = tolerance;
        return this;
    }

    /**
     * Set the upper and lower power clamps for the rotator.
     *
     * @param lowerPower the lower power clamp
     * @param upperPower the upper power clamp
     * @return this
     */
    public Rotator withPowerClamps(double lowerPower, double upperPower) {
        LOWER_POWER = lowerPower;
        UPPER_POWER = upperPower;
        return this;
    }

    /**
     * Set the upper and lower angle limits for the rotator.
     *
     * @param min the min angle that can be achieved
     * @param max the max angle that can be achieved
     * @return this
     */
    public Rotator withAngleLimits(Measure<Angle> min, Measure<Angle> max) {
        MIN_LIMIT = min;
        MAX_LIMIT = max;
        return this;
    }

    /**
     * Set a target power for the rotator.
     *
     * @param targetPower target power
     */
    public void setPower(double targetPower) {
        power = Range.clip(targetPower, LOWER_POWER, UPPER_POWER);
    }

    /**
     * Set the current power of the rotator.
     *
     * @param targetPower the current targetPower
     * @return the task
     */
    public Task setPowerTask(double targetPower) {
        return new RunTask(() -> setPower(targetPower), this, true).withName("SetDegreesTask");
    }

    /**
     * Set a target power supplier for the rotator.
     *
     * @param targetPower target power
     * @return a task to set the power
     */
    public Task controlTask(DoubleSupplier targetPower) {
        return new ContinuousTask(() -> setPower(targetPower.getAsDouble()), this, false).withName("SetDegreesUsingControllerTask");
    }

    /**
     * Set the degrees target for the rotator, which will run until it reaches the target based on a timeout.
     *
     * @param angle   the target angle
     * @param runTime the time to allocate for the rotator to reach the target
     * @return the task
     */
    public Task gotoTimeTask(Measure<Angle> angle, Measure<Time> runTime) {
        return new Task(runTime, this, true) {
            @Override
            protected void init() {
                lockout = true;
                pivot.set(angle);
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivot.setPower(1.0);
            }

            @Override
            protected void periodic() {
                // no-op
            }

            @Override
            protected void onFinish() {
                lockout = false;
                pivot.holdCurrentPosition();
            }

            @Override
            protected boolean isTaskFinished() {
                return false;
            }
        }.withName("SetDegreesTimeTask");
    }

    /**
     * Set the degrees target for the rotator, which will run until it reaches the target based on the encoder.
     *
     * @param angle the target angle
     * @return the task
     */
    public Task gotoTask(Measure<Angle> angle) {
        return new NoTimeoutTask(this, true) {
            @Override
            protected void init() {
                lockout = true;
            }

            @Override
            protected void periodic() {
                pivot.set(angle);
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivot.setPower(1.0);
            }

            @Override
            protected void onFinish() {
                lockout = false;
                pivot.holdCurrentPosition();
            }

            @Override
            protected boolean isTaskFinished() {
                return !pivot.isBusy() && Math.abs(angle.minus(pivot.getCurrent()).in(Degrees)) < TOLERANCE.in(Degrees);
            }
        }.withName("SetDegreesGotoTask");
    }

    /**
     * Run the rotator for a certain amount of time with certain power.
     * This can be used as a homing task, passing resetAfter as true will reset the encoder after the task.
     *
     * @param time        the time to run for
     * @param targetPower the power to run at
     * @param resetAfter  whether to reset the rotator encoder after the task
     * @return the task
     */
    public Task runForTask(Measure<Time> time, double targetPower, boolean resetAfter) {
        return new Task(time, this, true) {
            @Override
            protected void init() {
                lockout = true;
                pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pivot.setPower(Range.clip(targetPower, LOWER_POWER, UPPER_POWER));
            }

            @Override
            protected void periodic() {
                // no-op
            }

            @Override
            protected void onFinish() {
                pivot.setPower(0);
                if (resetAfter)
                    pivot.reset();
                pivot.holdCurrentPosition();
                lockout = false;
            }

            @Override
            protected boolean isTaskFinished() {
                // Based on timeout
                return false;
            }
        }.withName("RunForTask");
    }

    @Override
    protected void periodic() {
        if (lockout) return;

        if ((pivot.getCurrent().lt(MIN_LIMIT) && power < 0.0) || (pivot.getCurrent().gt(MAX_LIMIT) && power > 0.0))
            power = 0.0;

        if (power == 0.0) {
            // Hold the rotator in place
            pivot.holdCurrentPosition();
        } else {
            // Use user inputs
            pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pivot.setPower(power);
        }

        opMode.addTelemetry("%: % <= % <= % degs, % pwr", NAME, MIN_LIMIT, round(pivot.getCurrent().in(Degrees), 1), MAX_LIMIT, power);
    }
}
