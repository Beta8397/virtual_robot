package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;

/**
 * Wrapper class for a DcMotor where all motor speed is passed through a configurable SmoothDamp function.
 *
 * @author Lucas Bubner, 2024
 */
public class DcMotorRamping extends DcMotorImplEx {
    private final ElapsedTime timer = new ElapsedTime();
    private final Reference<Double> v = Reference.of(0.0);

    // Ramping enabled by default
    private boolean enabled = true;
    private Measure<Time> smoothTime = Seconds.of(0.1);
    private double maxDelta = 1.0;

    ///////// VIRTUAL ROBOT CHANGES /////////
    private final DcMotor VIRTUAL_MOTOR;
    /**
     * Create a new DcMotorRamping object, wrapping a DcMotor object and
     * returning a new object that can be used in place of the original.
     * By default, the ramping function time is set to 0.1 seconds, with a maximum delta of 1.0 power units per second.
     *
     * @param motor the DcMotor object to wrap. By default, the ramping function is enabled.
     */
    public DcMotorRamping(DcMotor motor) {
        super(motor.getController(), motor.getPortNumber());
        this.VIRTUAL_MOTOR = motor;
    }
    /**
     * Create a new DcMotorRamping object, wrapping a DcMotor object and
     * returning a new object that can be used in place of the original.
     *
     * @param motor the DcMotor object to wrap. By default, the ramping function is enabled.
     * @param smoothTime the time it takes for the motor to reach the target power level
     * @param maxDelta the maximum change in power level per second
     */
    public DcMotorRamping(DcMotor motor, Measure<Time> smoothTime, double maxDelta) {
        super(motor.getController(), motor.getPortNumber());
        this.smoothTime = smoothTime;
        this.maxDelta = maxDelta;
        this.VIRTUAL_MOTOR = motor;
    }
    @Override public double getPower() { return VIRTUAL_MOTOR.getPower(); }
    @Override public void setDirection(Direction d) { VIRTUAL_MOTOR.setDirection(d); }
    /////////////////////////////////////////

    /**
     * Set whether the ramping function is enabled.
     * If disabled, the motor will instantly set its power level to the target power level.
     *
     * @param enabled whether the ramping function is enabled, default is on
     */
    public void setRampingEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /**
     * Set the ramping parameters of the motor.
     *
     * @param smoothTime the time it takes for the motor to reach the target power level
     * @param maxDelta the maximum change in power level per second
     */
    public void setRampingParameters(Measure<Time> smoothTime, double maxDelta) {
        this.smoothTime = smoothTime;
        this.maxDelta = maxDelta;
    }

    /**
     * Set the time it takes for the motor to reach the target power level.
     *
     * @param smoothTime the time it takes for the motor to reach the target power level
     */
    public void setRampingTime(Measure<Time> smoothTime) {
        this.smoothTime = smoothTime;
    }

    /**
     * Set the maximum change in power level per second.
     *
     * @param maxDelta the maximum change in power level per second
     */
    public void setMaxDelta(double maxDelta) {
        this.maxDelta = maxDelta;
    }

    /**
     * Set the power level of the motor, which will be passed through a SmoothDamp function defined by the motor's ramping parameters.
     * <b>This function must be called periodically</b> (e.g. constantly during an activeLoop) to update the motor's power level,
     * as it relies on the time since the last call to calculate the new power level.
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     */
    @Override
    public void setPower(double power) {
        ///////////// VIRTUAL ROBOT CHANGES /////////////
        if (!enabled) {
            VIRTUAL_MOTOR.setPower(power);
            return;
        }

        if (VIRTUAL_MOTOR.getPower() == power) {
            timer.reset();
            return;
        }

        VIRTUAL_MOTOR.setPower(Mathf.smoothDamp(VIRTUAL_MOTOR.getPower(), power, v, smoothTime, maxDelta, deltaTime()));
        /////////////////////////////////////////////////
    }

    private Measure<Time> deltaTime() {
        double dt = timer.seconds();
        timer.reset();
        return Seconds.of(dt);
    }

    /**
     * Instantly set the power level of the motor, bypassing the SmoothDamp function.
     */
    public void setPowerInstant(double power) {
        super.setPower(power);
    }

    /**
     * Instantly stop the motor, bypassing the SmoothDamp function.
     */
    public void stop() {
        setPowerInstant(0);
    }
}
