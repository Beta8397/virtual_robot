package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

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
public class ThreeWheelTrackingLocalizer extends ThreeTrackingWheelLocalizer {
    private final ThreeWheelTrackingLocalizerCoefficients coefficients;
    private final Deadwheel leftDeadwheel;
    private final Deadwheel rightDeadwheel;
    private final Deadwheel frontDeadwheel;
    private final List<Integer> lastEncPositions;
    private final List<Integer> lastEncVels;
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
    public ThreeWheelTrackingLocalizer(ThreeWheelTrackingLocalizerCoefficients coefficients, Deadwheel leftDeadwheel, Deadwheel rightDeadwheel, Deadwheel frontDeadwheel, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
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
    }

    /**
     * Enable overflow compensation if your encoders exceed 32767 counts / second.
     *
     * @return this
     */
    public ThreeWheelTrackingLocalizer enableOverflowCompensation() {
        usingOverflowCompensation = true;
        return this;
    }

    public ThreeWheelTrackingLocalizerCoefficients getCoefficients() {
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
                encoderTicksToInches(leftPos),
                encoderTicksToInches(rightPos),
                encoderTicksToInches(frontPos)
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
                encoderTicksToInches(leftVel),
                encoderTicksToInches(rightVel),
                encoderTicksToInches(frontVel)
        );
    }
}
