package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.NonNull;

import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;

import java.util.function.DoubleSupplier;

/**
 * A RampingValue that uses a SmoothDamp function to smooth out values over time.
 * Uses a supplier for the current value, based on a target set by the loop.
 *
 * @author Lucas Bubner, 2024
 */
public class RampingSupplier implements RampingFunction {
    private final RampingValue v = new RampingValue();
    private DoubleSupplier current;

    /**
     * Create a new RampingValue object, wrapping a DoubleSupplier that will be used
     * in place of the current value when {@link #get(double)} is called.
     *
     * @param current the DoubleSupplier object to wrap. By default, the ramping function is enabled.
     */
    public RampingSupplier(@NonNull DoubleSupplier current) {
        this.current = current;
    }

    public RampingSupplier setCurrentSupplier(@NonNull DoubleSupplier current) {
        this.current = current;
        return this;
    }

    /**
     * Set whether the ramping function is enabled.
     * If disabled, the value will instantly set its value to the target value.
     *
     * @param enabled whether the ramping function is enabled, default is on
     */
    @Override
    public RampingSupplier setRampingEnabled(boolean enabled) {
        v.setRampingEnabled(enabled);
        return this;
    }

    /**
     * Set the ramping parameters of the value.
     *
     * @param time  the time it takes for the value to reach the target value.
     * @param delta the maximum change in value per second
     * @return this
     */
    @Override
    public RampingSupplier setRampingParameters(Measure<Time> time, double delta) {
        v.setRampingParameters(time, delta);
        return this;
    }

    /**
     * Set the time it takes for the value to reach the target.
     *
     * @param smoothTime the time it takes for the value to reach the target.
     * @return this
     */
    @Override
    public RampingSupplier setRampingTime(Measure<Time> smoothTime) {
        v.setRampingTime(smoothTime);
        return this;
    }

    /**
     * Set the maximum change in units per second.
     *
     * @param maxDelta the maximum change in units per second
     * @return this
     */
    @Override
    public RampingSupplier setMaxDelta(double maxDelta) {
        v.setMaxDelta(maxDelta);
        return this;
    }

    /**
     * Gets a ramped value from the value supplier and target value.
     * Must be called regularly to update the SmoothDamp velocities.
     *
     * @param target the target value
     * @return a ramped result
     */
    public double get(double target) {
        return v.get(current.getAsDouble(), target);
    }


    /**
     * Gets a ramped value from the value supplier and target value.
     * Must be called regularly to update the SmoothDamp velocities.
     *
     * @param target the target value
     * @return a ramped result
     */
    public float get(float target) {
        return (float) get((double) target);
    }
}
