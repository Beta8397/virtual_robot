package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import org.murraybridgebunyips.bunyipslib.roadrunner.util.Encoder;

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
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    private final StandardTrackingWheelLocalizerCoefficients coefficients;
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder frontEncoder;
    private final List<Integer> lastEncPositions;
    private final List<Integer> lastEncVels;

    /**
     * Create a new StandardTrackingWheelLocalizer with coefficients, encoders, and last encoder positions and velocities.
     *
     * @param coefficients             The coefficients for the localizer
     * @param leftEncoder              The left encoder
     * @param rightEncoder             The right encoder
     * @param frontEncoder             The front encoder
     * @param lastTrackingEncPositions The last encoder positions
     * @param lastTrackingEncVels      The last encoder velocities
     */
    public StandardTrackingWheelLocalizer(StandardTrackingWheelLocalizerCoefficients coefficients, Encoder leftEncoder, Encoder rightEncoder, Encoder frontEncoder, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, coefficients.LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -coefficients.LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(coefficients.FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        this.coefficients = coefficients;

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        assert leftEncoder != null && rightEncoder != null && frontEncoder != null;

        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.frontEncoder = frontEncoder;
    }

    public StandardTrackingWheelLocalizerCoefficients getCoefficients() {
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
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

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
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

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
