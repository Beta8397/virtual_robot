package org.murraybridgebunyips.bunyipslib;

import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;

/**
 * A ramping function that can be used to smooth out values over time.
 *
 * @author Lucas Bubner, 2024
 */
public interface RampingFunction {
    /**
     * Set whether the ramping function is enabled.
     * If disabled, the value will instantly set its value to the target value.
     *
     * @param enabled whether the ramping function is enabled, default is on
     */
    RampingFunction setRampingEnabled(boolean enabled);
    /**
     * Set the ramping parameters of the value.
     *
     * @param time the time it takes for the value to reach the target value.
     * @param delta the maximum change in value per second
     * @return this
     */
    RampingFunction setRampingParameters(Measure<Time> time, double delta);
    /**
     * Set the time it takes for the value to reach the target.
     *
     * @param smoothTime the time it takes for the value to reach the target.
     * @return this
     */
    RampingFunction setRampingTime(Measure<Time> smoothTime);
    /**
     * Set the maximum change in units per second.
     *
     * @param maxDelta the maximum change in units per second
     * @return this
     */
    RampingFunction setMaxDelta(double maxDelta);
}
