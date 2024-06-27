package org.murraybridgebunyips.bunyipslib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;

/**
 * Wrapper class for a DcMotorEx where all motor speed is passed through a configurable SmoothDamp function.
 *
 * @author Lucas Bubner, 2024
 */
public class DcMotorRamping extends DcMotorImplEx implements RampingFunction {
    private final RampingSupplier v = new RampingSupplier(this::getPower);

    /**
     * Create a new DcMotorRamping object, wrapping a DcMotor object and
     * returning a new object that can be used in place of the original.
     * By default, the ramping function time is set to 0.1 seconds, with a maximum delta of 1.0 power units per second.
     *
     * @param motor the DcMotor object to wrap. By default, the ramping function is enabled.
     */
    public DcMotorRamping(DcMotor motor) {
        super(motor.getController(), motor.getPortNumber());
    }

    /**
     * Create a new DcMotorRamping object, wrapping a DcMotor object and
     * returning a new object that can be used in place of the original.
     *
     * @param motor      the DcMotor object to wrap. By default, the ramping function is enabled.
     * @param smoothTime the time it takes for the motor to reach the target power level
     * @param maxDelta   the maximum change in power level per second
     */
    public DcMotorRamping(DcMotor motor, Measure<Time> smoothTime, double maxDelta) {
        super(motor.getController(), motor.getPortNumber());
        v.setRampingParameters(smoothTime, maxDelta);
    }

    /**
     * Set whether the ramping function is enabled.
     * If disabled, the motor will instantly set its power level to the target power level.
     *
     * @param enabled whether the ramping function is enabled, default is on
     */
    @Override
    public DcMotorRamping setRampingEnabled(boolean enabled) {
        v.setRampingEnabled(enabled);
        return this;
    }

    /**
     * Set the ramping parameters of the motor.
     *
     * @param smoothTime the time it takes for the motor to reach the target power level
     * @param maxDelta   the maximum change in power level per second
     */
    @Override
    public DcMotorRamping setRampingParameters(Measure<Time> smoothTime, double maxDelta) {
        v.setRampingParameters(smoothTime, maxDelta);
        return this;
    }

    /**
     * Set the time it takes for the motor to reach the target power level.
     *
     * @param smoothTime the time it takes for the motor to reach the target power level
     */
    @Override
    public DcMotorRamping setRampingTime(Measure<Time> smoothTime) {
        v.setRampingTime(smoothTime);
        return this;
    }

    /**
     * Set the maximum change in power level per second.
     *
     * @param maxDelta the maximum change in power level per second
     */
    @Override
    public DcMotorRamping setMaxDelta(double maxDelta) {
        v.setMaxDelta(maxDelta);
        return this;
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
        super.setPower(v.get(power));
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
