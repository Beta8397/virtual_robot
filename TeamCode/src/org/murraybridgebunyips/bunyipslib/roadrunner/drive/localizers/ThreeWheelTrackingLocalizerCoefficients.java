package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;

import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;

/**
 * Constants for RoadRunner standard tracking wheels.
 * Reworked to use a builder for multiple robot configurations.
 *
 * @author Lucas Bubner, 2023
 */
public class ThreeWheelTrackingLocalizerCoefficients {

    /**
     * Ticks per revolution of the tracking wheel encoder.
     */
    public double TICKS_PER_REV;
    /**
     * Radius of the tracking wheel.
     */
    public double WHEEL_RADIUS = 2; // in
    /**
     * Gear ratio of the tracking wheel.
     */
    public double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    /**
     * Inches of lateral distance between the left and right wheels.
     */
    public double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    /**
     * Forward offset of the lateral wheel to the center of rotation.
     */
    public double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    /**
     * Utility builder for creating new coefficients.
     */
    public static class Builder {

        private final ThreeWheelTrackingLocalizerCoefficients trackingWheelCoefficients;

        /**
         * Start building.
         */
        public Builder() {
            trackingWheelCoefficients = new ThreeWheelTrackingLocalizerCoefficients();
        }

        /**
         * Set the ticks per revolution of the tracking wheel encoder.
         *
         * @param ticksPerRev The ticks per revolution
         * @return The builder
         */
        public Builder setTicksPerRev(double ticksPerRev) {
            trackingWheelCoefficients.TICKS_PER_REV = ticksPerRev;
            return this;
        }

        /**
         * Set the radius of the tracking wheel.
         *
         * @param wheelRadius The radius of the tracking wheel
         * @return The builder
         */
        public Builder setWheelRadius(Measure<Distance> wheelRadius) {
            trackingWheelCoefficients.WHEEL_RADIUS = wheelRadius.in(Inches);
            return this;
        }

        /**
         * Set the gear ratio of the tracking wheel.
         *
         * @param gearRatio The gear ratio of the tracking wheel (output wheel speed / input encoder speed)
         * @return The builder
         */
        public Builder setGearRatio(double gearRatio) {
            trackingWheelCoefficients.GEAR_RATIO = gearRatio;
            return this;
        }

        /**
         * Set the lateral distance between the left and right tracking wheels.
         *
         * @param lateralDistance The lateral distance between the left and right wheels
         * @return The builder
         */
        public Builder setLateralDistance(Measure<Distance> lateralDistance) {
            trackingWheelCoefficients.LATERAL_DISTANCE = lateralDistance.in(Inches);
            return this;
        }

        /**
         * Set the forward offset of the lateral tracking wheel to the center of rotation.
         *
         * @param forwardOffset The forward offset of the lateral tracking wheel
         * @return The builder
         */
        public Builder setForwardOffset(Measure<Distance> forwardOffset) {
            trackingWheelCoefficients.FORWARD_OFFSET = forwardOffset.in(Inches);
            return this;
        }

        /**
         * Build the coefficients.
         *
         * @return The coefficients
         */
        public ThreeWheelTrackingLocalizerCoefficients build() {
            return trackingWheelCoefficients;
        }
    }
}
