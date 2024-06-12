package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;

/**
 * SmoothDamp implementation for a double value.
 *
 * @author Lucas Bubner, 2024
 */
public class RampingValue implements RampingFunction {
    private final ElapsedTime timer = new ElapsedTime();
    private final Reference<Double> v = Reference.of(0.0);
    private Measure<Time> smoothTime = Seconds.of(0.1);
    private double maxDelta = 1.0;
    // Enabled by default
    private boolean enabled = true;

    /**
     * Set whether the ramping function is enabled.
     * If disabled, the value will instantly set its value to the target value.
     *
     * @param enabled whether the ramping function is enabled, default is on
     */
    @Override
    public RampingValue setRampingEnabled(boolean enabled) {
        this.enabled = enabled;
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
    public RampingValue setRampingParameters(Measure<Time> time, double delta) {
        smoothTime = time;
        maxDelta = delta;
        return this;
    }

    /**
     * Set the time it takes for the value to reach the target.
     *
     * @param smoothTime the time it takes for the value to reach the target.
     */
    @Override
    public RampingValue setRampingTime(Measure<Time> smoothTime) {
        this.smoothTime = smoothTime;
        return this;
    }

    /**
     * Set the maximum change in units per second.
     *
     * @param maxDelta the maximum change in units per second
     */
    @Override
    public RampingValue setMaxDelta(double maxDelta) {
        this.maxDelta = maxDelta;
        return this;
    }

    /**
     * Gets a SmoothDamped result from the value supplier.
     * Must be called regularly to update the SmoothDamp velocities.
     *
     * @param current the current value
     * @param target  the target value
     * @return a result
     */
    public double get(double current, double target) {
        if (!enabled)
            return target;

        if (current == target) {
            timer.reset();
            return target;
        }

        return Mathf.smoothDamp(current, target, v, smoothTime, maxDelta, deltaTime());
    }

    /**
     * Gets a SmoothDamped result from the value supplier.
     * Must be called regularly to update the SmoothDamp velocities.
     *
     * @param current the current value
     * @param target  the target value
     * @return a result
     */
    public float get(float current, float target) {
        return (float) get((double) current, target);
    }

    private Measure<Time> deltaTime() {
        double dt = timer.seconds();
        timer.reset();
        return Seconds.of(dt);
    }
}
