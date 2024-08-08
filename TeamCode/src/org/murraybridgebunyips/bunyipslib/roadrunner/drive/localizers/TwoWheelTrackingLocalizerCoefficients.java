package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;

import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;

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
     * Multiplicative scale of the ticks from the x (forward) axis.
     */
    public double X_MULTIPLIER = 1;
    /**
     * Multiplicative scale of the ticks from the y (strafe) axis.
     */
    public double Y_MULTIPLIER = 1;

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

        /**
         * Set the ticks per revolution of the tracking wheel encoder.
         *
         * @param ticksPerRev The ticks per revolution
         * @return The builder
         */
        public Builder setTicksPerRev(double ticksPerRev) {
            twoWheelTrackingCoefficients.TICKS_PER_REV = ticksPerRev;
            return this;
        }

        /**
         * Set the radius of the tracking wheel.
         *
         * @param wheelRadius The radius of the tracking wheel
         * @return The builder
         */
        public Builder setWheelRadius(Measure<Distance> wheelRadius) {
            twoWheelTrackingCoefficients.WHEEL_RADIUS = wheelRadius.in(Inches);
            return this;
        }

        /**
         * Set the gear ratio of the tracking wheel.
         *
         * @param gearRatio The gear ratio of the tracking wheel (output wheel speed / input encoder speed)
         * @return The builder
         */
        public Builder setGearRatio(double gearRatio) {
            twoWheelTrackingCoefficients.GEAR_RATIO = gearRatio;
            return this;
        }

        /**
         * Set the position of the parallel wheel in the forward direction.
         *
         * @param parallelX The position of the parallel wheel in the forward direction from the center of rotation
         * @return The builder
         */
        public Builder setParallelX(Measure<Distance> parallelX) {
            twoWheelTrackingCoefficients.PARALLEL_X = parallelX.in(Inches);
            return this;
        }

        /**
         * Set the position of the parallel wheel in the strafe direction.
         *
         * @param parallelY The position of the parallel wheel in the strafe direction from the center of rotation
         * @return The builder
         */
        public Builder setParallelY(Measure<Distance> parallelY) {
            twoWheelTrackingCoefficients.PARALLEL_Y = parallelY.in(Inches);
            return this;
        }

        /**
         * Set the position of the perpendicular wheel in the forward direction.
         *
         * @param perpendicularX The position of the perpendicular wheel in the forward direction from the center of rotation
         * @return The builder
         */
        public Builder setPerpendicularX(Measure<Distance> perpendicularX) {
            twoWheelTrackingCoefficients.PERPENDICULAR_X = perpendicularX.in(Inches);
            return this;
        }

        /**
         * Set the position of the perpendicular wheel in the strafe direction.
         *
         * @param perpendicularY The position of the perpendicular wheel in the strafe direction from the center of rotation
         * @return The builder
         */
        public Builder setPerpendicularY(Measure<Distance> perpendicularY) {
            twoWheelTrackingCoefficients.PERPENDICULAR_Y = perpendicularY.in(Inches);
            return this;
        }

        /**
         * Set the forward (x) multiplier for the ticks reported by the forward deadwheels.
         *
         * @param forwardMul the multiplier
         * @return The builder
         */
        public Builder setXMultiplier(double forwardMul) {
            twoWheelTrackingCoefficients.X_MULTIPLIER = forwardMul;
            return this;
        }

        /**
         * Set the strafe (y) multiplier for the ticks reported by the side deadwheel.
         *
         * @param strafeMul the multiplier
         * @return The builder
         */
        public Builder setYMultiplier(double strafeMul) {
            twoWheelTrackingCoefficients.Y_MULTIPLIER = strafeMul;
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
