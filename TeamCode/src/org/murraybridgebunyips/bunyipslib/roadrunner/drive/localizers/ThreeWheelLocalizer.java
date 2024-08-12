package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.roadrunner.util.Deadwheel;

import java.util.Arrays;
import java.util.List;

/**
 * Standard 3 tracking wheel localizer implementation.
 */
/*
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 */
public class ThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {
    private final Coefficients coefficients;
    private final Deadwheel leftDeadwheel;
    private final Deadwheel rightDeadwheel;
    private final Deadwheel frontDeadwheel;
    private final List<Integer> lastEncPositions;
    private final List<Integer> lastEncVels;
    private final double xMul;
    private final double yMul;
    private boolean usingOverflowCompensation;

    /**
     * Create a new StandardTrackingWheelLocalizer with coefficients, encoders, and last encoder positions and velocities.
     *
     * @param coefficients             The coefficients for the localizer
     * @param leftDeadwheel            The left encoder
     * @param rightDeadwheel           The right encoder
     * @param frontDeadwheel           The front encoder
     * @param lastTrackingEncPositions The last encoder positions
     * @param lastTrackingEncVels      The last encoder velocities
     */
    public ThreeWheelLocalizer(Coefficients coefficients, Deadwheel leftDeadwheel, Deadwheel rightDeadwheel, Deadwheel frontDeadwheel, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, coefficients.LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -coefficients.LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(coefficients.FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        this.coefficients = coefficients;

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        assert leftDeadwheel != null && rightDeadwheel != null && frontDeadwheel != null;

        this.leftDeadwheel = leftDeadwheel;
        this.rightDeadwheel = rightDeadwheel;
        this.frontDeadwheel = frontDeadwheel;

        xMul = coefficients.X_MULTIPLIER;
        yMul = coefficients.Y_MULTIPLIER;
    }

    /**
     * Enable overflow compensation if your encoders exceed 32767 counts / second.
     *
     * @return this
     */
    public ThreeWheelLocalizer enableOverflowCompensation() {
        usingOverflowCompensation = true;
        return this;
    }

    public Coefficients getCoefficients() {
        return coefficients;
    }

    /**
     * Convert encoder ticks to inches.
     *
     * @param ticks The encoder ticks
     * @return The inches traveled
     */
    public double encoderTicksToInches(double ticks) {
        return coefficients.WHEEL_RADIUS * 2 * Math.PI * coefficients.GEAR_RATIO * ticks / coefficients.TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftDeadwheel.getCurrentPosition();
        int rightPos = rightDeadwheel.getCurrentPosition();
        int frontPos = frontDeadwheel.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos) * xMul,
                encoderTicksToInches(rightPos) * xMul,
                encoderTicksToInches(frontPos) * yMul
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = usingOverflowCompensation ? (int) leftDeadwheel.getCorrectedVelocity() : (int) leftDeadwheel.getRawVelocity();
        int rightVel = usingOverflowCompensation ? (int) rightDeadwheel.getCorrectedVelocity() : (int) rightDeadwheel.getRawVelocity();
        int frontVel = usingOverflowCompensation ? (int) frontDeadwheel.getCorrectedVelocity() : (int) frontDeadwheel.getRawVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel) * xMul,
                encoderTicksToInches(rightVel) * xMul,
                encoderTicksToInches(frontVel) * yMul
        );
    }

    /**
     * Constants for RoadRunner standard tracking wheels.
     * Reworked to use a builder for multiple robot configurations.
     *
     * @author Lucas Bubner, 2023
     */
    public static class Coefficients {
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
         * Multiplicative scale of the ticks from the x (forward) axis.
         */
        public double X_MULTIPLIER = 1;
        /**
         * Multiplicative scale of the ticks from the y (strafe) axis.
         */
        public double Y_MULTIPLIER = 1;

        /**
         * Utility builder for creating new coefficients.
         */
        public static class Builder {

            private final Coefficients trackingWheelCoefficients;

            /**
             * Start building.
             */
            public Builder() {
                trackingWheelCoefficients = new Coefficients();
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
             * Set the forward (x) multiplier for the ticks reported by the forward deadwheels.
             *
             * @param forwardMul the multiplier
             * @return The builder
             */
            public Builder setXMultiplier(double forwardMul) {
                trackingWheelCoefficients.X_MULTIPLIER = forwardMul;
                return this;
            }

            /**
             * Set the strafe (y) multiplier for the ticks reported by the side deadwheel.
             *
             * @param strafeMul the multiplier
             * @return The builder
             */
            public Builder setYMultiplier(double strafeMul) {
                trackingWheelCoefficients.Y_MULTIPLIER = strafeMul;
                return this;
            }

            /**
             * Build the coefficients.
             *
             * @return The coefficients
             */
            public Coefficients build() {
                return trackingWheelCoefficients;
            }
        }
    }
}
