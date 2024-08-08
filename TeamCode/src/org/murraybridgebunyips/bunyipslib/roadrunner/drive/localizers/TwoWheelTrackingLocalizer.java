package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;

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
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    private final TwoWheelTrackingLocalizerCoefficients coefficients;

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
    public TwoWheelTrackingLocalizer(TwoWheelTrackingLocalizerCoefficients coefficients, Deadwheel parallelDeadwheel, Deadwheel perpendicularDeadwheel, RoadRunnerDrive drive) {
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
    public TwoWheelTrackingLocalizer enableOverflowCompensation() {
        usingOverflowCompensation = true;
        return this;
    }

    public TwoWheelTrackingLocalizerCoefficients getCoefficients() {
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
}