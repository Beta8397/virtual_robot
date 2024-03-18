package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

/**
 * Coefficients for RoadRunner two wheel tracking localizer.
 * Reworked to use a builder for multiple robot configurations.
 *
 * @author Lucas Bubner, 2023
 */
public class TwoWheelTrackingLocalizerCoefficients {
    /**
     * The number of ticks per revolution of the encoder
     */
    public double TICKS_PER_REV;
    /**
     * The radius of the tracking wheel in inches
     */
    public double WHEEL_RADIUS = 2; // in
    /**
     * The gear ratio of the tracking wheel, (output wheel speed / input encoder speed)
     */
    public double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    /**
     * Position in the forward direction of the parallel wheel in inches
     */
    public double PARALLEL_X; // X is the up and down direction
    /**
     * Position in the strafe direction of the parallel wheel in inches
     */
    public double PARALLEL_Y; // Y is the strafe direction

    /**
     * Position in the forward direction of the perpendicular wheel in inches
     */
    public double PERPENDICULAR_X;
    /**
     * Position in the strafe direction of the perpendicular wheel in inches
     */
    public double PERPENDICULAR_Y;

    /**
     * Utility class for building TwoWheelTrackingLocalizerCoefficients
     */
    public static class Builder {

        private final TwoWheelTrackingLocalizerCoefficients twoWheelTrackingCoefficients;

        /**
         * Start building.
         */
        public Builder() {
            twoWheelTrackingCoefficients = new TwoWheelTrackingLocalizerCoefficients();
        }

        public Builder setTicksPerRev(double ticksPerRev) {
            twoWheelTrackingCoefficients.TICKS_PER_REV = ticksPerRev;
            return this;
        }

        public Builder setWheelRadius(double wheelRadius) {
            twoWheelTrackingCoefficients.WHEEL_RADIUS = wheelRadius;
            return this;
        }

        public Builder setGearRatio(double gearRatio) {
            twoWheelTrackingCoefficients.GEAR_RATIO = gearRatio;
            return this;
        }

        public Builder setParallelX(double parallelX) {
            twoWheelTrackingCoefficients.PARALLEL_X = parallelX;
            return this;
        }

        public Builder setParallelY(double parallelY) {
            twoWheelTrackingCoefficients.PARALLEL_Y = parallelY;
            return this;
        }

        public Builder setPerpendicularX(double perpendicularX) {
            twoWheelTrackingCoefficients.PERPENDICULAR_X = perpendicularX;
            return this;
        }

        public Builder setPerpendicularY(double perpendicularY) {
            twoWheelTrackingCoefficients.PERPENDICULAR_Y = perpendicularY;
            return this;
        }

        /**
         * Build the coefficients.
         *
         * @return The coefficients
         */
        public TwoWheelTrackingLocalizerCoefficients build() {
            return twoWheelTrackingCoefficients;
        }
    }

}
