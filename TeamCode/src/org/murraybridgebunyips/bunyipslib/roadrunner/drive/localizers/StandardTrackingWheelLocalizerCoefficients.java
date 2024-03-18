package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

/**
 * Constants for RoadRunner standard tracking wheels.
 * Reworked to use a builder for multiple robot configurations.
 *
 * @author Lucas Bubner, 2023
 */
public class StandardTrackingWheelLocalizerCoefficients {

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

        private final StandardTrackingWheelLocalizerCoefficients trackingWheelCoefficients;

        /**
         * Start building.
         */
        public Builder() {
            trackingWheelCoefficients = new StandardTrackingWheelLocalizerCoefficients();
        }

        public Builder setTicksPerRev(double ticksPerRev) {
            trackingWheelCoefficients.TICKS_PER_REV = ticksPerRev;
            return this;
        }

        public Builder setWheelRadius(double wheelRadius) {
            trackingWheelCoefficients.WHEEL_RADIUS = wheelRadius;
            return this;
        }

        public Builder setGearRatio(double gearRatio) {
            trackingWheelCoefficients.GEAR_RATIO = gearRatio;
            return this;
        }

        public Builder setLateralDistance(double lateralDistance) {
            trackingWheelCoefficients.LATERAL_DISTANCE = lateralDistance;
            return this;
        }

        public Builder setForwardOffset(double forwardOffset) {
            trackingWheelCoefficients.FORWARD_OFFSET = forwardOffset;
            return this;
        }

        /**
         * Build the coefficients.
         *
         * @return The coefficients
         */
        public StandardTrackingWheelLocalizerCoefficients build() {
            return trackingWheelCoefficients;
        }
    }
}
