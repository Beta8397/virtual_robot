package org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.PathContinuityViolationException;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.DisplacementMarker;
import com.acmerobotics.roadrunner.trajectory.DisplacementProducer;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.SpatialMarker;
import com.acmerobotics.roadrunner.trajectory.TemporalMarker;
import com.acmerobotics.roadrunner.trajectory.TimeProducer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.Angle;

import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.sequencesegment.SequenceSegment;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.sequencesegment.TrajectorySegment;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.sequencesegment.TurnSegment;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.sequencesegment.WaitSegment;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * RoadRunner Trajectory Builder
 *
 * @param <T> The type of the subclass (used for fluent builder pattern)
 */
@SuppressWarnings("unchecked")
public class TrajectorySequenceBuilder<T extends TrajectorySequenceBuilder<T>> {
    private final double resolution = 0.25;

    private final TrajectoryVelocityConstraint baseVelConstraint;
    private final TrajectoryAccelerationConstraint baseAccelConstraint;
    private final double baseTurnConstraintMaxAngVel;
    private final double baseTurnConstraintMaxAngAccel;
    private final List<SequenceSegment> sequenceSegments;
    private final List<TemporalMarker> temporalMarkers;
    private final List<DisplacementMarker> displacementMarkers;
    private final List<SpatialMarker> spatialMarkers;
    private TrajectoryVelocityConstraint currentVelConstraint;
    private TrajectoryAccelerationConstraint currentAccelConstraint;
    private double currentTurnConstraintMaxAngVel;
    private double currentTurnConstraintMaxAngAccel;
    private Pose2d lastPose;

    private double tangentOffset;

    private boolean setAbsoluteTangent;
    private double absoluteTangent;

    private TrajectoryBuilder currentTrajectoryBuilder;

    private double currentDuration;
    private double currentDisplacement;

    private double lastDurationTraj;
    private double lastDisplacementTraj;

    /**
     * Create a new TrajectorySequenceBuilder
     *
     * @param startPose                     The starting pose (in, in, rad)
     * @param startTangent                  The starting tangent (rad)
     * @param baseVelConstraint             The base velocity constraint (inches/sec)
     * @param baseAccelConstraint           The base acceleration constraint (inches/sec^2)
     * @param baseTurnConstraintMaxAngVel   The base turn constraint maximum angular velocity (radians/sec)
     * @param baseTurnConstraintMaxAngAccel The base turn constraint maximum angular acceleration (radians/sec^2)
     */
    public TrajectorySequenceBuilder(
            Pose2d startPose,
            Double startTangent,
            TrajectoryVelocityConstraint baseVelConstraint,
            TrajectoryAccelerationConstraint baseAccelConstraint,
            double baseTurnConstraintMaxAngVel,
            double baseTurnConstraintMaxAngAccel
    ) {
        this.baseVelConstraint = baseVelConstraint;
        this.baseAccelConstraint = baseAccelConstraint;

        currentVelConstraint = baseVelConstraint;
        currentAccelConstraint = baseAccelConstraint;

        this.baseTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel;
        this.baseTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel;

        currentTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel;
        currentTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel;

        sequenceSegments = new ArrayList<>();

        temporalMarkers = new ArrayList<>();
        displacementMarkers = new ArrayList<>();
        spatialMarkers = new ArrayList<>();

        lastPose = startPose;

        tangentOffset = 0.0;

        setAbsoluteTangent = (startTangent != null);
        absoluteTangent = startTangent != null ? startTangent : 0.0;

        currentTrajectoryBuilder = null;

        currentDuration = 0.0;
        currentDisplacement = 0.0;

        lastDurationTraj = 0.0;
        lastDisplacementTraj = 0.0;
    }

    /**
     * Create a new TrajectorySequenceBuilder
     *
     * @param startPose                     The starting pose (in, in, rad)
     * @param baseVelConstraint             The base velocity constraint (inches/sec)
     * @param baseAccelConstraint           The base acceleration constraint (inches/sec^2)
     * @param baseTurnConstraintMaxAngVel   The base turn constraint maximum angular velocity (radians/sec)
     * @param baseTurnConstraintMaxAngAccel The base turn constraint maximum angular acceleration (radians/sec^2)
     */
    public TrajectorySequenceBuilder(
            Pose2d startPose,
            TrajectoryVelocityConstraint baseVelConstraint,
            TrajectoryAccelerationConstraint baseAccelConstraint,
            double baseTurnConstraintMaxAngVel,
            double baseTurnConstraintMaxAngAccel
    ) {
        this(
                startPose, null,
                baseVelConstraint, baseAccelConstraint,
                baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel
        );
    }

    public double getBaseTurnConstraintMaxAngVel() {
        return baseTurnConstraintMaxAngVel;
    }

    public double getBaseTurnConstraintMaxAngAccel() {
        return baseTurnConstraintMaxAngAccel;
    }

    public TrajectoryVelocityConstraint getBaseVelConstraint() {
        return baseVelConstraint;
    }

    public TrajectoryAccelerationConstraint getBaseAccelConstraint() {
        return baseAccelConstraint;
    }

    /**
     * Move in a straight line to a given position.
     *
     * @param endPosition The end position (inches)
     * @return The builder
     */
    public T lineTo(Vector2d endPosition) {
        return addPath(() -> currentTrajectoryBuilder.lineTo(endPosition, currentVelConstraint, currentAccelConstraint));
    }

    /**
     * Move in a straight line to a given position with custom velocity and acceleration constraints.
     *
     * @param endPosition     The end position (inches)
     * @param velConstraint   The velocity constraint (inches/sec)
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T lineTo(
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.lineTo(endPosition, velConstraint, accelConstraint));
    }

    /**
     * Move in a straight line to a given position with a constant heading.
     *
     * @param endPosition The end position (inches)
     * @return The builder
     */
    public T lineToConstantHeading(Vector2d endPosition) {
        return addPath(() -> currentTrajectoryBuilder.lineToConstantHeading(endPosition, currentVelConstraint, currentAccelConstraint));
    }

    /**
     * Move in a straight line to a given position with a constant heading and custom velocity and acceleration constraints.
     *
     * @param endPosition     The end position (inches)
     * @param velConstraint   The velocity constraint (inches/sec)
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T lineToConstantHeading(
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.lineToConstantHeading(endPosition, velConstraint, accelConstraint));
    }

    /**
     * Move in a straight line to a given position with a linear heading.
     *
     * @param endPose The end pose (in, in, rad)
     * @return The builder
     */
    public T lineToLinearHeading(Pose2d endPose) {
        return addPath(() -> currentTrajectoryBuilder.lineToLinearHeading(endPose, currentVelConstraint, currentAccelConstraint));
    }

    /**
     * Move in a straight line to a given position with a linear heading and custom velocity and acceleration constraints.
     *
     * @param endPose         The end pose (in, in, rad)
     * @param velConstraint   The velocity constraint (inches/sec)
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T lineToLinearHeading(
            Pose2d endPose,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.lineToLinearHeading(endPose, velConstraint, accelConstraint));
    }

    /**
     * Move in a straight line to a given position with a spline heading.
     *
     * @param endPose The end pose (in, in, rad)
     * @return The builder
     */
    public T lineToSplineHeading(Pose2d endPose) {
        return addPath(() -> currentTrajectoryBuilder.lineToSplineHeading(endPose, currentVelConstraint, currentAccelConstraint));
    }

    /**
     * Move in a straight line to a given position with a spline heading and custom velocity and acceleration constraints.
     *
     * @param endPose         The end pose (in, in, rad)
     * @param velConstraint   The velocity constraint (inches/sec)
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T lineToSplineHeading(
            Pose2d endPose,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.lineToSplineHeading(endPose, velConstraint, accelConstraint));
    }

    /**
     * Move in a strafe straight line to a given position.
     *
     * @param endPosition The end position (inches)
     * @return The builder
     */
    public T strafeTo(Vector2d endPosition) {
        return addPath(() -> currentTrajectoryBuilder.strafeTo(endPosition, currentVelConstraint, currentAccelConstraint));
    }

    /**
     * Move in a strafe straight line to a given position with custom velocity and acceleration constraints.
     *
     * @param endPosition     The end position (inches)
     * @param velConstraint   The velocity constraint (inches/sec)
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T strafeTo(
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.strafeTo(endPosition, velConstraint, accelConstraint));
    }

    /**
     * Move forward a given distance.
     *
     * @param distance The distance to move (inches)
     * @return The builder
     */
    public T forward(double distance) {
        return addPath(() -> currentTrajectoryBuilder.forward(distance, currentVelConstraint, currentAccelConstraint));
    }

    /**
     * Move forward a given distance with custom velocity and acceleration constraints.
     *
     * @param distance        The distance to move (inches)
     * @param velConstraint   The velocity constraint (inches/sec)
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T forward(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.forward(distance, velConstraint, accelConstraint));
    }

    /**
     * Move backward a given distance.
     *
     * @param distance The distance to move (inches)
     * @return The builder
     */
    public T back(double distance) {
        return addPath(() -> currentTrajectoryBuilder.back(distance, currentVelConstraint, currentAccelConstraint));
    }

    /**
     * Move backward a given distance with custom velocity and acceleration constraints.
     *
     * @param distance        The distance to move (inches)
     * @param velConstraint   The velocity constraint (inches/sec)
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T back(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.back(distance, velConstraint, accelConstraint));
    }

    /**
     * Strafe left a given distance.
     *
     * @param distance The distance to strafe (inches)
     * @return The builder
     */
    public T strafeLeft(double distance) {
        return addPath(() -> currentTrajectoryBuilder.strafeLeft(distance, currentVelConstraint, currentAccelConstraint));
    }

    /**
     * Strafe left a given distance with custom velocity and acceleration constraints.
     *
     * @param distance        The distance to strafe (inches)
     * @param velConstraint   The velocity constraint (inches/sec)
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T strafeLeft(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.strafeLeft(distance, velConstraint, accelConstraint));
    }

    /**
     * Strafe right a given distance.
     *
     * @param distance The distance to strafe (inches)
     * @return The builder
     */
    public T strafeRight(double distance) {
        return addPath(() -> currentTrajectoryBuilder.strafeRight(distance, currentVelConstraint, currentAccelConstraint));
    }

    /**
     * Strafe right a given distance with custom velocity and acceleration constraints.
     *
     * @param distance        The distance to strafe (inches)
     * @param velConstraint   The velocity constraint (inches/sec)
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T strafeRight(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.strafeRight(distance, velConstraint, accelConstraint));
    }

    /**
     * Spline to a given position with a given heading.
     *
     * @param endPosition The end position (inches)
     * @param endHeading  The end heading (rad)
     * @return The builder
     */
    public T splineTo(Vector2d endPosition, double endHeading) {
        return addPath(() -> currentTrajectoryBuilder.splineTo(endPosition, endHeading, currentVelConstraint, currentAccelConstraint));
    }

    /**
     * Spline to a given position with a given heading and custom velocity and acceleration constraints.
     *
     * @param endPosition     The end position (inches)
     * @param endHeading      The end heading (rad)
     * @param velConstraint   The velocity constraint (inches/sec)
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T splineTo(
            Vector2d endPosition,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.splineTo(endPosition, endHeading, velConstraint, accelConstraint));
    }

    /**
     * Spline to a given position with a constant heading.
     *
     * @param endPosition The end position (inches)
     * @param endHeading  The end heading (rad)
     * @return The builder
     */
    public T splineToConstantHeading(Vector2d endPosition, double endHeading) {
        return addPath(() -> currentTrajectoryBuilder.splineToConstantHeading(endPosition, endHeading, currentVelConstraint, currentAccelConstraint));
    }

    /**
     * Spline to a given position with a constant heading and custom velocity and acceleration constraints.
     *
     * @param endPosition     The end position (inches)
     * @param endHeading      The end heading (rad)
     * @param velConstraint   The velocity constraint (inches/sec)
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T splineToConstantHeading(
            Vector2d endPosition,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.splineToConstantHeading(endPosition, endHeading, velConstraint, accelConstraint));
    }

    /**
     * Spline to a given position with a linear heading.
     *
     * @param endPose    The end pose (in, in, rad)
     * @param endHeading The end heading (rad)
     * @return The builder
     */
    public T splineToLinearHeading(Pose2d endPose, double endHeading) {
        return addPath(() -> currentTrajectoryBuilder.splineToLinearHeading(endPose, endHeading, currentVelConstraint, currentAccelConstraint));
    }

    /**
     * Spline to a given position with a linear heading and custom velocity and acceleration constraints.
     *
     * @param endPose         The end pose (in, in, rad)
     * @param endHeading      The end heading (rad)
     * @param velConstraint   The velocity constraint (inches/sec)
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T splineToLinearHeading(
            Pose2d endPose,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.splineToLinearHeading(endPose, endHeading, velConstraint, accelConstraint));
    }

    /**
     * Spline to a given position with a spline heading.
     *
     * @param endPose    The end pose (in, in, rad)
     * @param endHeading The end heading (rad)
     * @return The builder
     */
    public T splineToSplineHeading(Pose2d endPose, double endHeading) {
        return addPath(() -> currentTrajectoryBuilder.splineToSplineHeading(endPose, endHeading, currentVelConstraint, currentAccelConstraint));
    }

    /**
     * Spline to a given position with a spline heading and custom velocity and acceleration constraints.
     *
     * @param endPose         The end pose (in, in, rad)
     * @param endHeading      The end heading (rad)
     * @param velConstraint   The velocity constraint (inches/sec)
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T splineToSplineHeading(
            Pose2d endPose,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.splineToSplineHeading(endPose, endHeading, velConstraint, accelConstraint));
    }

    private T addPath(Runnable callback) {
        if (currentTrajectoryBuilder == null) newPath();

        try {
            callback.run();
        } catch (PathContinuityViolationException e) {
            newPath();
            callback.run();
        }

        Trajectory builtTraj = currentTrajectoryBuilder.build();

        double durationDifference = builtTraj.duration() - lastDurationTraj;
        double displacementDifference = builtTraj.getPath().length() - lastDisplacementTraj;

        lastPose = builtTraj.end();
        currentDuration += durationDifference;
        currentDisplacement += displacementDifference;

        lastDurationTraj = builtTraj.duration();
        lastDisplacementTraj = builtTraj.getPath().length();

        return (T) this;
    }

    public T setTangent(double tangent) {
        setAbsoluteTangent = true;
        absoluteTangent = tangent;

        pushPath();

        return (T) this;
    }

    private T setTangentOffset(double offset) {
        setAbsoluteTangent = false;

        tangentOffset = offset;
        pushPath();

        return (T) this;
    }

    /**
     * Reverse or unreverse the tangent of the next path.
     *
     * @param reversed Whether to reverse the tangent
     * @return The builder
     */
    public T setReversed(boolean reversed) {
        return reversed ? setTangentOffset(Math.PI) : setTangentOffset(0.0);
    }

    /**
     * Set the velocity and acceleration constraints for the next builder instructions.
     *
     * @param velConstraint   The velocity constraint (inches/sec)
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T setConstraints(
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        currentVelConstraint = velConstraint;
        currentAccelConstraint = accelConstraint;

        return (T) this;
    }

    /**
     * Reset the velocity and acceleration constraints to the base constraints.
     *
     * @return The builder
     */
    public T resetConstraints() {
        currentVelConstraint = baseVelConstraint;
        currentAccelConstraint = baseAccelConstraint;

        return (T) this;
    }

    /**
     * Set the velocity constraint for the next builder instructions.
     *
     * @param velConstraint The velocity constraint (inches/sec)
     * @return The builder
     */
    public T setVelConstraint(TrajectoryVelocityConstraint velConstraint) {
        currentVelConstraint = velConstraint;

        return (T) this;
    }

    /**
     * Reset the velocity constraint to the base velocity constraint.
     *
     * @return The builder
     */
    public T resetVelConstraint() {
        currentVelConstraint = baseVelConstraint;

        return (T) this;
    }

    /**
     * Set the acceleration constraint for the next builder instructions.
     *
     * @param accelConstraint The acceleration constraint (inches/sec^2)
     * @return The builder
     */
    public T setAccelConstraint(TrajectoryAccelerationConstraint accelConstraint) {
        currentAccelConstraint = accelConstraint;

        return (T) this;
    }

    /**
     * Reset the acceleration constraint to the base acceleration constraint.
     *
     * @return The builder
     */
    public T resetAccelConstraint() {
        currentAccelConstraint = baseAccelConstraint;

        return (T) this;
    }

    /**
     * Set the turn constraints for the next builder instructions.
     *
     * @param maxAngVel   The maximum angular velocity (radians/sec)
     * @param maxAngAccel The maximum angular acceleration (radians/sec^2)
     * @return The builder
     */
    public T setTurnConstraint(double maxAngVel, double maxAngAccel) {
        currentTurnConstraintMaxAngVel = maxAngVel;
        currentTurnConstraintMaxAngAccel = maxAngAccel;

        return (T) this;
    }

    /**
     * Reset the turn constraints to the base turn constraints.
     *
     * @return The builder
     */
    public T resetTurnConstraint() {
        currentTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel;
        currentTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel;

        return (T) this;
    }

    /**
     * Add a temporal marker at the current duration to run a callback at that time.
     *
     * @param callback The callback to run
     * @return The builder
     */
    public T addTemporalMarker(MarkerCallback callback) {
        return addTemporalMarker(currentDuration, callback);
    }

    /**
     * Add a temporal marker at the current duration plus an offset to run a callback at that time.
     *
     * @param offset   The offset to add to the current duration (seconds)
     * @param callback The callback to run
     * @return The builder
     */
    public T UNSTABLE_addTemporalMarkerOffset(double offset, MarkerCallback callback) {
        return addTemporalMarker(currentDuration + offset, callback);
    }

    /**
     * Add a temporal marker at a given time to run a callback at that time.
     *
     * @param time     The time to run the callback (seconds)
     * @param callback The callback to run
     * @return The builder
     */
    public T addTemporalMarker(double time, MarkerCallback callback) {
        return addTemporalMarker(0.0, time, callback);
    }

    /**
     * Add a temporal marker at a given time to run a callback at that time.
     *
     * @param scale    A multiplicative scale to apply to the time
     * @param offset   The offset to add to the time (seconds)
     * @param callback The callback to run
     * @return The builder
     */
    public T addTemporalMarker(double scale, double offset, MarkerCallback callback) {
        return addTemporalMarker(time -> scale * time + offset, callback);
    }

    /**
     * Add a temporal marker at a given time to run a callback at that time.
     *
     * @param time     The time to run the callback (seconds)
     * @param callback The callback to run
     * @return The builder
     */
    public T addTemporalMarker(TimeProducer time, MarkerCallback callback) {
        temporalMarkers.add(new TemporalMarker(time, callback));
        return (T) this;
    }

    /**
     * Add a spatial marker at the current position to run a callback at that position.
     *
     * @param point    The point to run the callback (inches)
     * @param callback The callback to run
     * @return The builder
     */
    public T addSpatialMarker(Vector2d point, MarkerCallback callback) {
        spatialMarkers.add(new SpatialMarker(point, callback));
        return (T) this;
    }

    /**
     * Add a displacement marker at the current displacement to run a callback at that displacement.
     *
     * @param callback The callback to run
     * @return The builder
     */
    public T addDisplacementMarker(MarkerCallback callback) {
        return addDisplacementMarker(currentDisplacement, callback);
    }

    /**
     * Add a displacement marker at the current displacement plus an offset to run a callback at that displacement.
     *
     * @param offset   The offset to add to the current displacement (inches)
     * @param callback The callback to run
     * @return The builder
     */
    public T UNSTABLE_addDisplacementMarkerOffset(double offset, MarkerCallback callback) {
        return addDisplacementMarker(currentDisplacement + offset, callback);
    }

    /**
     * Add a displacement marker at a given displacement to run a callback at that displacement.
     *
     * @param displacement The displacement to run the callback (inches)
     * @param callback     The callback to run
     * @return The builder
     */
    public T addDisplacementMarker(double displacement, MarkerCallback callback) {
        return addDisplacementMarker(0.0, displacement, callback);
    }

    /**
     * Add a displacement marker at a given displacement to run a callback at that displacement.
     *
     * @param scale    A multiplicative scale to apply to the displacement
     * @param offset   The offset to add to the displacement (inches)
     * @param callback The callback to run
     * @return The builder
     */
    public T addDisplacementMarker(double scale, double offset, MarkerCallback callback) {
        return addDisplacementMarker((displacement -> scale * displacement + offset), callback);
    }

    /**
     * Add a displacement marker at a given displacement to run a callback at that displacement.
     *
     * @param displacement The displacement to run the callback (inches)
     * @param callback     The callback to run
     * @return The builder
     */
    public T addDisplacementMarker(DisplacementProducer displacement, MarkerCallback callback) {
        displacementMarkers.add(new DisplacementMarker(displacement, callback));

        return (T) this;
    }

    /**
     * Turn to a given angle.
     *
     * @param angle The angle to turn (radians)
     * @return The builder
     */
    public T turn(double angle) {
        return turn(angle, currentTurnConstraintMaxAngVel, currentTurnConstraintMaxAngAccel);
    }

    /**
     * Turn to a given angle with custom maximum angular velocity and acceleration.
     *
     * @param angle       The angle to turn (radians)
     * @param maxAngVel   The maximum angular velocity (radians/sec)
     * @param maxAngAccel The maximum angular acceleration (radians/sec^2)
     * @return The builder
     */
    public T turn(double angle, double maxAngVel, double maxAngAccel) {
        pushPath();

        MotionProfile turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(lastPose.getHeading(), 0.0, 0.0, 0.0),
                new MotionState(lastPose.getHeading() + angle, 0.0, 0.0, 0.0),
                maxAngVel,
                maxAngAccel
        );

        sequenceSegments.add(new TurnSegment(lastPose, angle, turnProfile, Collections.emptyList()));

        lastPose = new Pose2d(
                lastPose.getX(), lastPose.getY(),
                Angle.norm(lastPose.getHeading() + angle)
        );

        currentDuration += turnProfile.duration();

        return (T) this;
    }

    /**
     * Wait for a given number of seconds.
     *
     * @param seconds The number of seconds to wait
     * @return The builder
     */
    public T waitSeconds(double seconds) {
        pushPath();
        sequenceSegments.add(new WaitSegment(lastPose, seconds, Collections.emptyList()));

        currentDuration += seconds;
        return (T) this;
    }

    /**
     * Add a trajectory to the sequence.
     *
     * @param trajectory The trajectory to add
     * @return The builder
     */
    public T addTrajectory(Trajectory trajectory) {
        pushPath();

        sequenceSegments.add(new TrajectorySegment(trajectory));
        return (T) this;
    }

    private void pushPath() {
        if (currentTrajectoryBuilder != null) {
            Trajectory builtTraj = currentTrajectoryBuilder.build();
            sequenceSegments.add(new TrajectorySegment(builtTraj));
        }

        currentTrajectoryBuilder = null;
    }

    private void newPath() {
        if (currentTrajectoryBuilder != null)
            pushPath();

        lastDurationTraj = 0.0;
        lastDisplacementTraj = 0.0;

        double tangent = setAbsoluteTangent ? absoluteTangent : Angle.norm(lastPose.getHeading() + tangentOffset);

        currentTrajectoryBuilder = new TrajectoryBuilder(lastPose, tangent, currentVelConstraint, currentAccelConstraint, resolution);
    }

    /**
     * Create a TrajectorySequence from the current builder state.
     *
     * @return The TrajectorySequence to run.
     */
    public TrajectorySequence build() {
        pushPath();

        List<TrajectoryMarker> globalMarkers = convertMarkersToGlobal(
                sequenceSegments,
                temporalMarkers, displacementMarkers, spatialMarkers
        );
        projectGlobalMarkersToLocalSegments(globalMarkers, sequenceSegments);

        return new TrajectorySequence(sequenceSegments);
    }

    private List<TrajectoryMarker> convertMarkersToGlobal(
            List<? extends SequenceSegment> segments,
            List<TemporalMarker> temporal,
            List<DisplacementMarker> displacement,
            List<SpatialMarker> spatial
    ) {
        ArrayList<TrajectoryMarker> trajectoryMarkers = new ArrayList<>();

        // Convert temporal markers
        for (TemporalMarker marker : temporal) {
            trajectoryMarkers.add(
                    new TrajectoryMarker(marker.getProducer().produce(currentDuration), marker.getCallback())
            );
        }

        // Convert displacement markers
        for (DisplacementMarker marker : displacement) {
            double time = displacementToTime(
                    segments,
                    marker.getProducer().produce(currentDisplacement)
            );

            trajectoryMarkers.add(
                    new TrajectoryMarker(
                            time,
                            marker.getCallback()
                    )
            );
        }

        // Convert spatial markers
        for (SpatialMarker marker : spatial) {
            trajectoryMarkers.add(
                    new TrajectoryMarker(
                            pointToTime(segments, marker.getPoint()),
                            marker.getCallback()
                    )
            );
        }

        return trajectoryMarkers;
    }

    private void projectGlobalMarkersToLocalSegments(List<TrajectoryMarker> markers, List<? extends SequenceSegment> segments) {
        if (segments.isEmpty()) return;

        markers.sort(Comparator.comparingDouble(TrajectoryMarker::getTime));

        double timeOffset = 0.0;
        int markerIndex = 0;
        for (SequenceSegment segment : segments) {
            while (markerIndex < markers.size()) {
                TrajectoryMarker marker = markers.get(markerIndex);
                if (marker.getTime() >= timeOffset + segment.getDuration()) {
                    break;
                }

                segment.getMarkers().add(new TrajectoryMarker(
                        Math.max(0.0, marker.getTime()) - timeOffset, marker.getCallback()));
                ++markerIndex;
            }

            timeOffset += segment.getDuration();
        }

        SequenceSegment segment = segments.get(segments.size() - 1);
        while (markerIndex < markers.size()) {
            TrajectoryMarker marker = markers.get(markerIndex);
            segment.getMarkers().add(new TrajectoryMarker(segment.getDuration(), marker.getCallback()));
            ++markerIndex;
        }
    }

    // Taken from Road Runner's TrajectoryGenerator.displacementToTime() since it's private
    // Note: this assumes that the profile position is monotonic increasing
    private Double motionProfileDisplacementToTime(MotionProfile profile, double s) {
        double tLo = 0.0;
        double tHi = profile.duration();
        while (!(Math.abs(tLo - tHi) < 1.0e-6)) {
            double tMid = 0.5 * (tLo + tHi);
            if (profile.get(tMid).getX() > s) {
                tHi = tMid;
            } else {
                tLo = tMid;
            }
        }
        return 0.5 * (tLo + tHi);
    }

    private Double displacementToTime(List<? extends SequenceSegment> segments, double s) {
        double currentTime = 0.0;
        double currentDisplacement = 0.0;

        for (SequenceSegment segment : segments) {
            if (segment instanceof TrajectorySegment) {
                TrajectorySegment thisSegment = (TrajectorySegment) segment;

                double segmentLength = thisSegment.getTrajectory().getPath().length();

                if (currentDisplacement + segmentLength > s) {
                    double target = s - currentDisplacement;
                    double timeInSegment = motionProfileDisplacementToTime(
                            thisSegment.getTrajectory().getProfile(),
                            target
                    );

                    return currentTime + timeInSegment;
                } else {
                    currentDisplacement += segmentLength;
                }
            }

            currentTime += segment.getDuration();
        }

        return currentTime;
    }

    private Double pointToTime(List<? extends SequenceSegment> sequences, Vector2d point) {
        class ComparingPoints {
            private final double distanceToPoint;
            private final double totalDisplacement;
            private final double thisPathDisplacement;

            private ComparingPoints(double distanceToPoint, double totalDisplacement, double thisPathDisplacement) {
                this.distanceToPoint = distanceToPoint;
                this.totalDisplacement = totalDisplacement;
                this.thisPathDisplacement = thisPathDisplacement;
            }
        }

        List<ComparingPoints> projectedPoints = new ArrayList<>();

        for (SequenceSegment segment : sequences) {
            if (segment instanceof TrajectorySegment) {
                TrajectorySegment thisSegment = (TrajectorySegment) segment;

                double displacement = thisSegment.getTrajectory().getPath().project(point, 0.25);
                Vector2d projectedPoint = thisSegment.getTrajectory().getPath().get(displacement).vec();
                double distanceToPoint = point.minus(projectedPoint).norm();

                double totalDisplacement = 0.0;

                for (ComparingPoints comparingPoint : projectedPoints) {
                    totalDisplacement += comparingPoint.totalDisplacement;
                }

                totalDisplacement += displacement;

                projectedPoints.add(new ComparingPoints(distanceToPoint, displacement, totalDisplacement));
            }
        }

        ComparingPoints closestPoint = null;

        for (ComparingPoints comparingPoint : projectedPoints) {
            if (closestPoint == null) {
                closestPoint = comparingPoint;
                continue;
            }

            if (comparingPoint.distanceToPoint < closestPoint.distanceToPoint)
                closestPoint = comparingPoint;
        }

        assert closestPoint != null;
        return displacementToTime(sequences, closestPoint.thisPathDisplacement);
    }
}
