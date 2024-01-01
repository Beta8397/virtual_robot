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

    public T lineTo(Vector2d endPosition) {
        return addPath(() -> currentTrajectoryBuilder.lineTo(endPosition, currentVelConstraint, currentAccelConstraint));
    }

    public T lineTo(
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.lineTo(endPosition, velConstraint, accelConstraint));
    }

    public T lineToConstantHeading(Vector2d endPosition) {
        return addPath(() -> currentTrajectoryBuilder.lineToConstantHeading(endPosition, currentVelConstraint, currentAccelConstraint));
    }

    public T lineToConstantHeading(
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.lineToConstantHeading(endPosition, velConstraint, accelConstraint));
    }

    public T lineToLinearHeading(Pose2d endPose) {
        return addPath(() -> currentTrajectoryBuilder.lineToLinearHeading(endPose, currentVelConstraint, currentAccelConstraint));
    }

    public T lineToLinearHeading(
            Pose2d endPose,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.lineToLinearHeading(endPose, velConstraint, accelConstraint));
    }

    public T lineToSplineHeading(Pose2d endPose) {
        return addPath(() -> currentTrajectoryBuilder.lineToSplineHeading(endPose, currentVelConstraint, currentAccelConstraint));
    }

    public T lineToSplineHeading(
            Pose2d endPose,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.lineToSplineHeading(endPose, velConstraint, accelConstraint));
    }

    public T strafeTo(Vector2d endPosition) {
        return addPath(() -> currentTrajectoryBuilder.strafeTo(endPosition, currentVelConstraint, currentAccelConstraint));
    }

    public T strafeTo(
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.strafeTo(endPosition, velConstraint, accelConstraint));
    }

    public T forward(double distance) {
        return addPath(() -> currentTrajectoryBuilder.forward(distance, currentVelConstraint, currentAccelConstraint));
    }

    public T forward(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.forward(distance, velConstraint, accelConstraint));
    }

    public T back(double distance) {
        return addPath(() -> currentTrajectoryBuilder.back(distance, currentVelConstraint, currentAccelConstraint));
    }

    public T back(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.back(distance, velConstraint, accelConstraint));
    }

    public T strafeLeft(double distance) {
        return addPath(() -> currentTrajectoryBuilder.strafeLeft(distance, currentVelConstraint, currentAccelConstraint));
    }

    public T strafeLeft(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.strafeLeft(distance, velConstraint, accelConstraint));
    }

    public T strafeRight(double distance) {
        return addPath(() -> currentTrajectoryBuilder.strafeRight(distance, currentVelConstraint, currentAccelConstraint));
    }

    public T strafeRight(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.strafeRight(distance, velConstraint, accelConstraint));
    }

    public T splineTo(Vector2d endPosition, double endHeading) {
        return addPath(() -> currentTrajectoryBuilder.splineTo(endPosition, endHeading, currentVelConstraint, currentAccelConstraint));
    }

    public T splineTo(
            Vector2d endPosition,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.splineTo(endPosition, endHeading, velConstraint, accelConstraint));
    }

    public T splineToConstantHeading(Vector2d endPosition, double endHeading) {
        return addPath(() -> currentTrajectoryBuilder.splineToConstantHeading(endPosition, endHeading, currentVelConstraint, currentAccelConstraint));
    }

    public T splineToConstantHeading(
            Vector2d endPosition,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.splineToConstantHeading(endPosition, endHeading, velConstraint, accelConstraint));
    }

    public T splineToLinearHeading(Pose2d endPose, double endHeading) {
        return addPath(() -> currentTrajectoryBuilder.splineToLinearHeading(endPose, endHeading, currentVelConstraint, currentAccelConstraint));
    }

    public T splineToLinearHeading(
            Pose2d endPose,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.splineToLinearHeading(endPose, endHeading, velConstraint, accelConstraint));
    }

    public T splineToSplineHeading(Pose2d endPose, double endHeading) {
        return addPath(() -> currentTrajectoryBuilder.splineToSplineHeading(endPose, endHeading, currentVelConstraint, currentAccelConstraint));
    }

    public T splineToSplineHeading(
            Pose2d endPose,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> currentTrajectoryBuilder.splineToSplineHeading(endPose, endHeading, velConstraint, accelConstraint));
    }

    private T addPath(AddPathCallback callback) {
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

    public T setReversed(boolean reversed) {
        return reversed ? setTangentOffset(3.141592653589793) : setTangentOffset(0.0);
    }

    public T setConstraints(
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        currentVelConstraint = velConstraint;
        currentAccelConstraint = accelConstraint;

        return (T) this;
    }

    public T resetConstraints() {
        currentVelConstraint = baseVelConstraint;
        currentAccelConstraint = baseAccelConstraint;

        return (T) this;
    }

    public T setVelConstraint(TrajectoryVelocityConstraint velConstraint) {
        currentVelConstraint = velConstraint;

        return (T) this;
    }

    public T resetVelConstraint() {
        currentVelConstraint = baseVelConstraint;

        return (T) this;
    }

    public T setAccelConstraint(TrajectoryAccelerationConstraint accelConstraint) {
        currentAccelConstraint = accelConstraint;

        return (T) this;
    }

    public T resetAccelConstraint() {
        currentAccelConstraint = baseAccelConstraint;

        return (T) this;
    }

    public T setTurnConstraint(double maxAngVel, double maxAngAccel) {
        currentTurnConstraintMaxAngVel = maxAngVel;
        currentTurnConstraintMaxAngAccel = maxAngAccel;

        return (T) this;
    }

    public T resetTurnConstraint() {
        currentTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel;
        currentTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel;

        return (T) this;
    }

    public T addTemporalMarker(MarkerCallback callback) {
        return addTemporalMarker(currentDuration, callback);
    }

    public T UNSTABLE_addTemporalMarkerOffset(double offset, MarkerCallback callback) {
        return addTemporalMarker(currentDuration + offset, callback);
    }

    public T addTemporalMarker(double time, MarkerCallback callback) {
        return addTemporalMarker(0.0, time, callback);
    }

    public T addTemporalMarker(double scale, double offset, MarkerCallback callback) {
        return addTemporalMarker(time -> scale * time + offset, callback);
    }

    public T addTemporalMarker(TimeProducer time, MarkerCallback callback) {
        temporalMarkers.add(new TemporalMarker(time, callback));
        return (T) this;
    }

    public T addSpatialMarker(Vector2d point, MarkerCallback callback) {
        spatialMarkers.add(new SpatialMarker(point, callback));
        return (T) this;
    }

    public T addDisplacementMarker(MarkerCallback callback) {
        return addDisplacementMarker(currentDisplacement, callback);
    }

    public T UNSTABLE_addDisplacementMarkerOffset(double offset, MarkerCallback callback) {
        return addDisplacementMarker(currentDisplacement + offset, callback);
    }

    public T addDisplacementMarker(double displacement, MarkerCallback callback) {
        return addDisplacementMarker(0.0, displacement, callback);
    }

    public T addDisplacementMarker(double scale, double offset, MarkerCallback callback) {
        return addDisplacementMarker((displacement -> scale * displacement + offset), callback);
    }

    public T addDisplacementMarker(DisplacementProducer displacement, MarkerCallback callback) {
        displacementMarkers.add(new DisplacementMarker(displacement, callback));

        return (T) this;
    }

    public T turn(double angle) {
        return turn(angle, currentTurnConstraintMaxAngVel, currentTurnConstraintMaxAngAccel);
    }

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

    public T waitSeconds(double seconds) {
        pushPath();
        sequenceSegments.add(new WaitSegment(lastPose, seconds, Collections.emptyList()));

        currentDuration += seconds;
        return (T) this;
    }

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
            List<SequenceSegment> sequenceSegments,
            List<TemporalMarker> temporalMarkers,
            List<DisplacementMarker> displacementMarkers,
            List<SpatialMarker> spatialMarkers
    ) {
        ArrayList<TrajectoryMarker> trajectoryMarkers = new ArrayList<>();

        // Convert temporal markers
        for (TemporalMarker marker : temporalMarkers) {
            trajectoryMarkers.add(
                    new TrajectoryMarker(marker.getProducer().produce(currentDuration), marker.getCallback())
            );
        }

        // Convert displacement markers
        for (DisplacementMarker marker : displacementMarkers) {
            double time = displacementToTime(
                    sequenceSegments,
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
        for (SpatialMarker marker : spatialMarkers) {
            trajectoryMarkers.add(
                    new TrajectoryMarker(
                            pointToTime(sequenceSegments, marker.getPoint()),
                            marker.getCallback()
                    )
            );
        }

        return trajectoryMarkers;
    }

    private void projectGlobalMarkersToLocalSegments(List<TrajectoryMarker> markers, List<SequenceSegment> sequenceSegments) {
        if (sequenceSegments.isEmpty()) return;

        markers.sort(Comparator.comparingDouble(TrajectoryMarker::getTime));

        double timeOffset = 0.0;
        int markerIndex = 0;
        for (SequenceSegment segment : sequenceSegments) {
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

        SequenceSegment segment = sequenceSegments.get(sequenceSegments.size() - 1);
        while (markerIndex < markers.size()) {
            TrajectoryMarker marker = markers.get(markerIndex);
            segment.getMarkers().add(new TrajectoryMarker(segment.getDuration(), marker.getCallback()));
            ++markerIndex;
        }
    }

    // Taken from Road Runner's TrajectoryGenerator.displacementToTime() since it's private
    // note: this assumes that the profile position is monotonic increasing
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

    private Double displacementToTime(List<SequenceSegment> sequenceSegments, double s) {
        double currentTime = 0.0;
        double currentDisplacement = 0.0;

        for (SequenceSegment segment : sequenceSegments) {
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

    private Double pointToTime(List<SequenceSegment> sequenceSegments, Vector2d point) {
        class ComparingPoints {
            private final double distanceToPoint;
            private final double totalDisplacement;
            private final double thisPathDisplacement;

            public ComparingPoints(double distanceToPoint, double totalDisplacement, double thisPathDisplacement) {
                this.distanceToPoint = distanceToPoint;
                this.totalDisplacement = totalDisplacement;
                this.thisPathDisplacement = thisPathDisplacement;
            }
        }

        List<ComparingPoints> projectedPoints = new ArrayList<>();

        for (SequenceSegment segment : sequenceSegments) {
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

        return displacementToTime(sequenceSegments, closestPoint.thisPathDisplacement);
    }

    private interface AddPathCallback {
        void run();
    }
}
