package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;

import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.util.Deadwheel;

import java.util.Arrays;
import java.util.List;

/**
 * Dual tracking wheel localizer implementation.
 */
/*
 * Tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 */
public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer {
    private final Coefficients coefficients;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private final Deadwheel parallelDeadwheel;
    private final Deadwheel perpendicularDeadwheel;
    private final double xMul;
    private final double yMul;
    private final RoadRunnerDrive drive;
    private boolean usingOverflowCompensation;

    /**
     * Create a new TwoWheelTrackingLocalizer
     *
     * @param coefficients           The coefficients to use
     * @param parallelDeadwheel      The parallel encoder
     * @param perpendicularDeadwheel The perpendicular encoder
     * @param drive                  The drivetrain
     */
    public TwoWheelLocalizer(Coefficients coefficients, Deadwheel parallelDeadwheel, Deadwheel perpendicularDeadwheel, RoadRunnerDrive drive) {
        super(Arrays.asList(
                new Pose2d(coefficients.PARALLEL_X, coefficients.PARALLEL_Y, 0),
                new Pose2d(coefficients.PERPENDICULAR_X, coefficients.PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;
        this.coefficients = coefficients;

        this.parallelDeadwheel = parallelDeadwheel;
        this.perpendicularDeadwheel = perpendicularDeadwheel;

        xMul = coefficients.X_MULTIPLIER;
        yMul = coefficients.Y_MULTIPLIER;
    }

    /**
     * Enable overflow compensation if your encoders exceed 32767 counts / second.
     *
     * @return this
     */
    public TwoWheelLocalizer enableOverflowCompensation() {
        usingOverflowCompensation = true;
        return this;
    }

    public Coefficients getCoefficients() {
        return coefficients;
    }

    /**
     * Convert encoder ticks to inches
     *
     * @param ticks The ticks to convert
     * @return The inches traveled
     */
    public double encoderTicksToInches(double ticks) {
        return coefficients.WHEEL_RADIUS * 2 * Math.PI * coefficients.GEAR_RATIO * ticks / coefficients.TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelDeadwheel.getCurrentPosition()) * xMul,
                encoderTicksToInches(perpendicularDeadwheel.getCurrentPosition()) * yMul
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        // high resolution encoders), enable overflow compensation with enableOverflowCompensation.
        return Arrays.asList(
                encoderTicksToInches(usingOverflowCompensation ? parallelDeadwheel.getCorrectedVelocity() : parallelDeadwheel.getRawVelocity()) * xMul,
                encoderTicksToInches(usingOverflowCompensation ? perpendicularDeadwheel.getCorrectedVelocity() : perpendicularDeadwheel.getRawVelocity()) * yMul
        );
    }

    /**
     * Coefficients for RoadRunner two wheel tracking localizer.
     * Reworked to use a builder for multiple robot configurations.
     *
     * @author Lucas Bubner, 2023
     */
    public static class Coefficients {
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
         * Utility class for building TwoWheelLocalizer.Coefficients
         */
        public static class Builder {

            private final Coefficients twoWheelTrackingCoefficients;

            /**
             * Start building.
             */
            public Builder() {
                twoWheelTrackingCoefficients = new Coefficients();
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
            public Coefficients build() {
                return twoWheelTrackingCoefficients;
            }
        }
    }
}