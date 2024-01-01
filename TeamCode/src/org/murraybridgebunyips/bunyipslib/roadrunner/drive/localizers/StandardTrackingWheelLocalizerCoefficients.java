package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

/**
 * Constants for RoadRunner standard tracking wheels.
 * Reworked to use a builder for multiple robot configurations.
 *
 * @author Lucas Bubner, 2023
 */
public class StandardTrackingWheelLocalizerCoefficients {

    public double TICKS_PER_REV;
    public double WHEEL_RADIUS = 2; // in
    public double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    public static class Builder {

        private final StandardTrackingWheelLocalizerCoefficients trackingWheelCoefficients;

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

        public StandardTrackingWheelLocalizerCoefficients build() {
            return trackingWheelCoefficients;
        }
    }
}
