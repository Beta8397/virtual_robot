package org.murraybridgebunyips.bunyipslib.subsystems;

import static org.murraybridgebunyips.bunyipslib.Text.round;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Amps;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.DegreesPerSecond;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.RevolutionsPerSecond;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.units.Current;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.tasks.ContinuousTask;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.NoTimeoutTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.function.DoubleSupplier;

/**
 * Controls a generic holdable motor, that may be actuated by a user's input
 * but will hold its position when the input is released.
 *
 * @author Lucas Bubner, 2024
 */
public class HoldableActuator extends BunyipsSubsystem {
    // Power to hold the actuator in place
    private double HOLDING_POWER = 1.0;
    // Power to move the actuator when in auto mode
    private double MOVING_POWER = 0.7;
    // Number of greater than zero velocity hits required for Home Task
    private int ZERO_HIT_THRESHOLD = 30;
    // Overcurrent for Home Task
    private Measure<Current> OVERCURRENT = Amps.of(4);
    // Time of which the Home Task Overcurrent needs to be exceeded
    private Measure<Time> OVERCURRENT_TIME = Seconds.of(1.0);
    // Name of the actuator for telemetry
    private String NAME = "Actuator";
    // Encoder lower limit in ticks
    private long MIN_LIMIT = -Long.MAX_VALUE;
    // Encoder upper limit in ticks
    private long MAX_LIMIT = Long.MAX_VALUE;
    // Lower power clamp
    private double LOWER_POWER = -1.0;
    // Upper power clamp
    private double UPPER_POWER = 1.0;
    // Tolerance for the actuator in ticks, default 2
    private int TOLERANCE = 2;
    private DcMotorEx motor;
    private TouchSensor bottomSwitch;
    private boolean zeroed;
    private boolean userLatch;
    private double userPower;
    private double motorPower;
    private Mode inputMode = Mode.USER;

    /**
     * Create a new HoldableActuator.
     *
     * @param motor the motor to control
     */
    public HoldableActuator(DcMotorEx motor) {
        if (!assertParamsNotNull(motor)) return;
        this.motor = motor;
        // Assumes current arm position is the zero position, the user may home manually if required using setInitTask
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(HOLDING_POWER);
    }

    /**
     * Set the name of the actuator to display in telemetry.
     *
     * @param newName the name to set
     * @return this
     */
    public HoldableActuator withName(String newName) {
        NAME = newName;
        return this;
    }

    /**
     * Set the target tolerance of the actuator.
     *
     * @param tolerance    the tolerance to set in encoder ticks
     * @param applyToMotor whether to apply this tolerance to the motor as well as the task checks
     * @return this
     */
    public HoldableActuator withTolerance(int tolerance, boolean applyToMotor) {
        if (applyToMotor)
            motor.setTargetPositionTolerance(tolerance);
        TOLERANCE = tolerance;
        return this;
    }

    /**
     * Set the zero hit threshold to how many greater than or equal to zero velocity hits are required for the Home Task.
     * If the actuator has a continuous negative velocity of zero for this many hits, the Home Task will complete.
     *
     * @param threshold the new threshold of continuous hits of zero velocity to complete homing. Default is 30.
     * @return this
     * @see #disableHomingZeroHits()
     */
    public HoldableActuator withHomingZeroHits(int threshold) {
        ZERO_HIT_THRESHOLD = threshold;
        return this;
    }

    /**
     * Disable the zero hit threshold for the Home Task.
     *
     * @return this
     */
    public HoldableActuator disableHomingZeroHits() {
        return withHomingZeroHits(0);
    }

    /**
     * Set the overcurrent threshold for the Home Task.
     * If this current is reached during a Home Task, for the set duration set here, the home task will end.
     *
     * @param current the current which if exceeded in a Home Task will finish the reset. Default is 4A.
     * @param forTime the time the current must be exceeded for to finish the reset. Useful for filtering out momentary spikes. Default is 1s.
     * @return this
     * @see #disableHomingOvercurrent()
     */
    public HoldableActuator withHomingOvercurrent(Measure<Current> current, Measure<Time> forTime) {
        OVERCURRENT = current;
        OVERCURRENT_TIME = forTime;
        return this;
    }

    /**
     * Disable the overcurrent threshold for the Home Task.
     *
     * @return this
     */
    public HoldableActuator disableHomingOvercurrent() {
        return withHomingOvercurrent(Amps.of(0), Seconds.of(0));
    }

    /**
     * Set the bottom limit switch of the actuator to use in homing and encoder awareness.
     *
     * @param bottomLimitSwitch the limit switch to set as the bottom switch where the arm would be "homed"
     * @return this
     * @see #disableHomingZeroHits()
     * @see #disableHomingOvercurrent()
     */
    public HoldableActuator withBottomSwitch(TouchSensor bottomLimitSwitch) {
        bottomSwitch = bottomLimitSwitch;
        return this;
    }

    /**
     * Set the holding power of the actuator. Note: this power is clamped by the lower and upper power clamps.
     *
     * @param targetPower the power to set
     * @return this
     */
    public HoldableActuator withHoldingPower(double targetPower) {
        HOLDING_POWER = targetPower;
        return this;
    }

    /**
     * Set the moving power of the actuator, where a positive value will bring the arm upwards (away from bottom).
     * Note: this power is clamped by the lower and upper power clamps.
     *
     * @param targetPower the power to set
     * @return this
     */
    public HoldableActuator withMovingPower(double targetPower) {
        MOVING_POWER = targetPower;
        return this;
    }

    /**
     * Set the lower power clamp of the actuator.
     *
     * @param lowerPower the lower power clamp to set
     * @return this
     */
    public HoldableActuator withLowerPowerClamp(double lowerPower) {
        return withPowerClamps(lowerPower, UPPER_POWER);
    }

    /**
     * Set the upper power clamp of the actuator.
     *
     * @param upperPower the upper power clamp to set
     * @return this
     */
    public HoldableActuator withUpperPowerClamp(double upperPower) {
        return withPowerClamps(LOWER_POWER, upperPower);
    }

    /**
     * Set the lower and upper power clamps of the actuator.
     *
     * @param lowerPower the lower power clamp to set
     * @param upperPower the upper power clamp to set
     * @return this
     */
    public HoldableActuator withPowerClamps(double lowerPower, double upperPower) {
        LOWER_POWER = Mathf.clamp(lowerPower, -1, 1);
        UPPER_POWER = Mathf.clamp(upperPower, -1, 1);
        return this;
    }

    /**
     * Set the lower limit of the actuator.
     *
     * @param minLimit the lower limit to set in encoder ticks
     * @return this
     */
    public HoldableActuator withLowerLimit(long minLimit) {
        MIN_LIMIT = minLimit;
        return this;
    }

    /**
     * Set the upper limit of the actuator.
     *
     * @param maxLimit the upper limit to set in encoder ticks
     * @return this
     */
    public HoldableActuator withUpperLimit(long maxLimit) {
        MAX_LIMIT = maxLimit;
        return this;
    }

    /**
     * Set the encoder limits of the actuator.
     *
     * @param minLimit the lower limit of the actuator in encoder ticks
     * @param maxLimit the upper limit of the actuator in encoder ticks
     * @return this
     */
    public HoldableActuator withEncoderLimits(long minLimit, long maxLimit) {
        MIN_LIMIT = minLimit;
        MAX_LIMIT = maxLimit;
        return this;
    }

    /**
     * Move the actuator with a supplier of power. Should be a default task.
     *
     * @param powerSupplier the power value supplier
     * @return a task to move the actuator
     */
    public Task controlTask(DoubleSupplier powerSupplier) {
        return new ContinuousTask(() -> setPower(powerSupplier.getAsDouble()), this, false).withName("JoystickControlTask");
    }

    public HoldableActuator setPower(double p) {
        userPower = Mathf.clamp(p, LOWER_POWER, UPPER_POWER);
        return this;
    }

    /**
     * Set the power of the actuator.
     *
     * @param p the power to set
     * @return a task to set the power
     */
    public Task setPowerTask(double p) {
        return new RunTask(() -> setPower(p), this, false).withName("SetPowerTask");
    }

    /**
     * Run the actuator for a certain amount of time.
     *
     * @param p    the power to run at
     * @param time the time to run for
     * @return a task to run the actuator
     */
    public Task runForTask(double p, Measure<Time> time) {
        return new Task(time, this, true) {
            @Override
            public void init() {
                inputMode = Mode.USER;
            }

            @Override
            public void periodic() {
                // Will hijack the user power by constantly setting it
                userPower = p;
            }

            @Override
            public void onFinish() {
                userPower = 0;
            }

            @Override
            public boolean isTaskFinished() {
                return false;
            }
        }.withName("RunForTask");
    }

    /**
     * Home the actuator based on encoders against a hard stop or limit switch.
     *
     * @return a task to home the actuator
     */
    public Task homeTask() {
        return new NoTimeoutTask() {
            private ElapsedTime overcurrentTimer;
            private double previousAmpAlert;
            private double zeroHits;

            @Override
            protected void init() {
                zeroHits = 0;
                inputMode = Mode.HOMING;
                previousAmpAlert = motor.getCurrentAlert(CurrentUnit.AMPS);
                motor.setCurrentAlert(OVERCURRENT.in(Amps), CurrentUnit.AMPS);
            }

            @Override
            protected void periodic() {
                if (ZERO_HIT_THRESHOLD <= 0) return;
                if (motor.getVelocity() >= 0) {
                    zeroHits++;
                } else {
                    zeroHits = 0;
                }
            }

            @Override
            protected void onFinish() {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setCurrentAlert(previousAmpAlert, CurrentUnit.AMPS);
                inputMode = Mode.USER;
            }

            @Override
            protected boolean isTaskFinished() {
                boolean bottomedOut = bottomSwitch != null && bottomSwitch.isPressed();
                boolean velocityZeroed = ZERO_HIT_THRESHOLD > 0 && zeroHits >= ZERO_HIT_THRESHOLD;
                boolean overCurrent = OVERCURRENT.magnitude() > 0 && motor.isOverCurrent();
                if (OVERCURRENT_TIME.magnitude() > 0 && overCurrent && overcurrentTimer == null) {
                    overcurrentTimer = new ElapsedTime();
                } else if (!overCurrent) {
                    overcurrentTimer = null;
                }
                boolean sustainedOvercurrent = overcurrentTimer != null && overcurrentTimer.seconds() >= OVERCURRENT_TIME.in(Seconds);
                return inputMode != Mode.HOMING || (bottomedOut || velocityZeroed || sustainedOvercurrent);
            }
        }.withName("HomeTask");
    }

    /**
     * Set the position of the actuator.
     * <p></p>
     * Informally known as the Doinky-Rubber-Bandy Task
     *
     * @param targetPosition the position to set
     * @return a task to set the position
     */
    public Task gotoTask(int targetPosition) {
        return new NoTimeoutTask(this, true) {
            @Override
            public void init() {
                motor.setTargetPosition(targetPosition);
                inputMode = Mode.AUTO;
            }

            @Override
            public void periodic() {
                // no-op
            }

            @Override
            public void onFinish() {
                inputMode = Mode.USER;
            }

            @Override
            public boolean isTaskFinished() {
                return inputMode != Mode.AUTO || (!motor.isBusy() && Mathf.isNear(targetPosition, motor.getCurrentPosition(), TOLERANCE));
            }
        }.withName("RunToPositionTask");
    }

    /**
     * Delta the position of the actuator.
     *
     * @param deltaPosition the delta to add to the current position of the actuator
     * @return a task to delta the position
     */
    public Task deltaTask(int deltaPosition) {
        return new NoTimeoutTask(this, true) {
            private int target;

            @Override
            public void init() {
                target = motor.getCurrentPosition() + deltaPosition;
                motor.setTargetPosition(target);
                inputMode = Mode.AUTO;
            }

            @Override
            public void periodic() {
                // no-op
            }

            @Override
            public void onFinish() {
                inputMode = Mode.USER;
            }

            @Override
            public boolean isTaskFinished() {
                return inputMode != Mode.AUTO || (!motor.isBusy() && Mathf.isNear(target, motor.getCurrentPosition(), TOLERANCE));
            }
        }.withName("DeltaPositionTask");
    }

    @Override
    protected void periodic() {
        switch (inputMode) {
            case AUTO:
                // Paranoia safety guard to ensure the motor does not enter RUN_TO_POSITION mode without a target
                try {
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } catch (Exception e) {
                    // Cannot catch ActionNotSupportedException due to package protections, so we will handle a generic exception
                    // In the event this does fail, we will fallback to the default mode.
                    inputMode = Mode.USER;
                    break;
                }
                motorPower = MOVING_POWER;
                opMode.addTelemetry("%: <font color='#FF5F1F'>MOVING -> %/% ticks</font> [%rps]", NAME, motor.getTargetPosition(), motor.getCurrentPosition(), round(DegreesPerSecond.of(motor.getVelocity(AngleUnit.DEGREES)).in(RevolutionsPerSecond), 1));
                break;
            case HOMING:
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorPower = -MOVING_POWER;
                opMode.addTelemetry("%: <font color='yellow'><b>HOMING</b></font> [%rps]", NAME, round(DegreesPerSecond.of(motor.getVelocity(AngleUnit.DEGREES)).in(RevolutionsPerSecond), 1));
                break;
            case USER:
                if (userPower == 0.0) {
                    // Hold arm in place
                    if (!userLatch) {
                        motor.setTargetPosition(motor.getCurrentPosition());
                        userLatch = true;
                    }
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorPower = HOLDING_POWER;
                } else {
                    userLatch = false;
                    // Move arm in accordance with the user's input
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorPower = userPower;
                }
                opMode.addTelemetry("%: % at % ticks [%tps]", NAME, userPower == 0.0 ? "<font color='green'>HOLDING</font>" : "<font color='#FF5F1F'><b>MOVING</b></font>", motor.getCurrentPosition(), Math.round(motor.getVelocity()));
                break;
        }

        if (bottomSwitch != null) {
            if (bottomSwitch.isPressed() && motorPower < 0) {
                // Cancel and stop any tasks that would move the actuator out of bounds as defined by the limit switch
                inputMode = Mode.USER;
                motorPower = 0;
            }

            if (bottomSwitch.isPressed() && !zeroed) {
                DcMotor.RunMode prev = motor.getMode();
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setTargetPosition(0);
                motor.setMode(prev);
                // Ensure we only run the reset once every time the switch is pressed
                zeroed = true;
            }

            if (!bottomSwitch.isPressed())
                zeroed = false;
        }

        if ((motor.getCurrentPosition() < MIN_LIMIT && motorPower < 0.0) || (motor.getCurrentPosition() > MAX_LIMIT && motorPower > 0.0)) {
            // Cancel any tasks that would move the actuator out of bounds
            inputMode = Mode.USER;
            motorPower = 0.0;
        }

        motor.setPower(Mathf.clamp(motorPower, LOWER_POWER, UPPER_POWER));
    }

    private enum Mode {
        AUTO,
        HOMING,
        USER
    }
}
