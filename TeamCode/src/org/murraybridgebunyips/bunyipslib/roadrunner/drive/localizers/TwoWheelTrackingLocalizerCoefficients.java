package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

/**
 * Coefficients for RoadRunner two wheel tracking localizer.
 * Reworked to use a builder for multiple robot configurations.
 *
 * @author Lucas Bubner, 2023
 */
public class TwoWheelTrackingLocalizerCoefficients {
    public double TICKS_PER_REV;
    public double WHEEL_RADIUS = 2; // in
    public double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public double PARALLEL_X; // X is the up and down direction
    public double PARALLEL_Y; // Y is the strafe direction

    public double PERPENDICULAR_X;
    public double PERPENDICULAR_Y;

    public static class Builder {

        private final TwoWheelTrackingLocalizerCoefficients twoWheelTrackingCoefficients;

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

        public TwoWheelTrackingLocalizerCoefficients build() {
            return twoWheelTrackingCoefficients;
        }
    }

}
