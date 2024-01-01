package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import org.murraybridgebunyips.bunyipslib.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Standard tracking wheel localizer implementation assuming the standard configuration:
 *
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
