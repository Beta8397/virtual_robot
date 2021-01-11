package system.robot.roadrunner_util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import util.math.geometry.Point2D;

import java.util.function.Function;

public final class HALTrajectoryBuilder{
    private final TrajectoryBuilder trajectoryBuilder;
    private final CoordinateMode coordinateMode;
    public HALTrajectoryBuilder(TrajectoryBuilder trajectoryBuilder, CoordinateMode coordinateMode) {
        this.trajectoryBuilder = trajectoryBuilder;
        this.coordinateMode = coordinateMode;
    }

    @NotNull
    public final HALTrajectoryBuilder lineTo(@NotNull final Point2D endPosition, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.lineTo(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(endPosition.getX(), endPosition.getY(), 0)
                ).vec(),
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder lineTo(@NotNull Point2D endPosition) {
        trajectoryBuilder.lineTo(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(endPosition.getX(), endPosition.getY(), 0)
                ).vec()
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder lineToConstantHeading(@NotNull final Point2D endPosition, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.lineToConstantHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(endPosition.getX(), endPosition.getY(), 0)
                ).vec(),
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder lineToConstantHeading(@NotNull Point2D endPosition) {
        trajectoryBuilder.lineToConstantHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(endPosition.getX(), endPosition.getY(), 0)
                ).vec()
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder lineToLinearHeading(@NotNull final Pose2d endPose, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.lineToLinearHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(endPose),
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder lineToLinearHeading(@NotNull Pose2d endPose) {
        trajectoryBuilder.lineToLinearHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(endPose)
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder lineToSplineHeading(@NotNull final Pose2d endPose, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.lineToSplineHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(endPose),
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder lineToSplineHeading(@NotNull Pose2d endPose) {
        trajectoryBuilder.lineToSplineHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(endPose)
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder strafeTo(@NotNull final Point2D endPosition, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.strafeTo(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(endPosition.getX(), endPosition.getY(), 0)
                ).vec(),
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder strafeTo(@NotNull Point2D endPosition) {
        trajectoryBuilder.strafeTo(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(endPosition.getX(), endPosition.getY(), 0)
                ).vec()
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder forward(final double distance, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.forward(distance, velConstraintOverride, accelConstraintOverride);
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder forward(double distance) {
        trajectoryBuilder.forward(distance);
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder back(final double distance, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.back(distance, velConstraintOverride, accelConstraintOverride);
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder back(double distance) {
        trajectoryBuilder.back(distance);
        return this;
    }


    @NotNull
    public final HALTrajectoryBuilder strafeLeft(final double distance, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.strafeLeft(distance, velConstraintOverride, accelConstraintOverride);
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder strafeLeft(double distance) {
        trajectoryBuilder.strafeLeft(distance);
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder strafeRight(final double distance, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.strafeRight(distance, velConstraintOverride, accelConstraintOverride);
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder strafeRight(double distance) {
        trajectoryBuilder.strafeRight(distance);
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder splineTo(@NotNull final Point2D endPosition, final double endTangent, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.splineTo(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(endPosition.getX(), endPosition.getY(), 0)
                ).vec(),
                endTangent,
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder splineTo(@NotNull Point2D endPosition, double endTangent) {
        trajectoryBuilder.splineTo(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(endPosition.getX(), endPosition.getY(), 0)
                ).vec(),
                endTangent
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder splineToConstantHeading(@NotNull final Point2D endPosition, final double endTangent, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.splineToConstantHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(endPosition.getX(), endPosition.getY(), 0)
                ).vec(),
                endTangent,
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder splineToConstantHeading(@NotNull Point2D endPosition, double endTangent) {
        trajectoryBuilder.splineToConstantHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(endPosition.getX(), endPosition.getY(), 0)
                ).vec(),
                endTangent
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder splineToLinearHeading(@NotNull final Pose2d endPose, final double endTangent, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.splineToLinearHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(endPose),
                endTangent,
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder splineToLinearHeading(@NotNull Pose2d endPose, double endTangent) {
        trajectoryBuilder.splineToLinearHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(endPose),
                endTangent
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder splineToSplineHeading(@NotNull final Pose2d endPose, final double endTangent, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.splineToSplineHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(endPose),
                endTangent,
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder splineToSplineHeading(@NotNull Pose2d endPose, double endTangent) {
        trajectoryBuilder.splineToSplineHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(endPose),
                endTangent
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder addTemporalMarker(double time, @NotNull MarkerCallback callback) {
        trajectoryBuilder.addTemporalMarker(time, callback);
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder addTemporalMarker(final double scale, final double offset, @NotNull MarkerCallback callback) {
        trajectoryBuilder.addTemporalMarker(scale, offset, callback);
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder addTemporalMarker(@NotNull Function<? super Double, Double> time, @NotNull MarkerCallback callback) {
        trajectoryBuilder.addTemporalMarker(time::apply, callback);
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder addSpatialMarker(@NotNull Point2D point, @NotNull MarkerCallback callback) {
        trajectoryBuilder.addSpatialMarker(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(point.getX(), point.getY(), 0)
                ).vec(),
                callback
        );
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder addDisplacementMarker(@NotNull MarkerCallback callback) {
        trajectoryBuilder.addDisplacementMarker(callback);
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder addDisplacementMarker(double displacement, @NotNull MarkerCallback callback) {
        trajectoryBuilder.addDisplacementMarker(displacement, callback);
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder addDisplacementMarker(final double scale, final double offset, @NotNull MarkerCallback callback) {
        trajectoryBuilder.addDisplacementMarker(scale, offset, callback);
        return this;
    }

    @NotNull
    public final HALTrajectoryBuilder addDisplacementMarker(@NotNull Function<? super Double, Double> displacement, @NotNull MarkerCallback callback) {
        trajectoryBuilder.addDisplacementMarker(displacement::apply, callback);
        return this;
    }

    @NotNull
    public final HALTrajectory build() {
        return new HALTrajectory(trajectoryBuilder.build(), coordinateMode);
    }
}
