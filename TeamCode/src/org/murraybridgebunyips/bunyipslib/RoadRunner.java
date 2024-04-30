package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;
import static org.murraybridgebunyips.bunyipslib.tasks.bases.Task.INFINITE_TIMEOUT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.DisplacementProducer;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.TimeProducer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.external.units.Velocity;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.murraybridgebunyips.bunyipslib.tasks.RoadRunnerTask;
import org.murraybridgebunyips.bunyipslib.tasks.groups.TaskGroup;

import java.util.ArrayList;

/**
 * RoadRunner utility interface for autonomous OpModes. Implement this interface in a {@link AutonomousBunyipsOpMode}.
 * Do not override any of the default methods in this interface, as they are used for RoadRunner task scheduling.
 * <p>
 * Previously named RoadRunnerAutonomousBunyipsOpMode (RRABOM, nickname "Rabone").
 * <p>
 * <i>This interface may also be used in a normal {@link BunyipsOpMode}, however all the {@code addTrajectory} related methods
 * will not work as they require the presence of {@link AutonomousBunyipsOpMode}.</i>
 *
 * @author Lucas Bubner, 2024
 * @noinspection InterfaceMayBeAnnotatedFunctional
 * @see AutonomousBunyipsOpMode
 */
public interface RoadRunner extends RoadRunnerDriveInstance {
    /**
     * Default timeout for all RoadRunner tasks, if not explicitly mentioned.
     */
    Measure<Time> DEFAULT_TIMEOUT = INFINITE_TIMEOUT;
    /**
     * A list of all RoadRunner tasks that have been scheduled using the RoadRunner methods.
     * Will be auto-cleared at the start of an {@link AutonomousBunyipsOpMode}.
     */
    ArrayList<RoadRunnerTask<RoadRunnerDrive>> rrTasks = new ArrayList<>();

    /**
     * Get the last known pose of the drive system, or the last pose from the last trajectory.
     *
     * @return Last known pose of the scheduled tasks or current pose of the drive, in inches
     */
    default Pose2d getPreviousPose() {
        // Needed to splice the last pose from the last trajectory
        return rrTasks.isEmpty() ? getDrive().getPoseEstimate() : rrTasks.get(rrTasks.size() - 1).getEndPose();
    }

    /**
     * Use this method to build a new RoadRunner trajectory or to add a RoadRunner trajectory to the task queue.
     *
     * @param startPoseInchRad Starting pose of the trajectory, <b>WILL SET DRIVE POSE ESTIMATE TO THIS POSE</b>, (in, in, radians)
     * @return Builder for the trajectory
     */
    default RoadRunnerTrajectoryTaskBuilder makeTrajectory(Pose2d startPoseInchRad) {
        // noinspection rawtypes
        TrajectorySequenceBuilder builder = getDrive().trajectorySequenceBuilder(startPoseInchRad);
        getDrive().setPoseEstimate(startPoseInchRad);
        return new RoadRunnerTrajectoryTaskBuilder(getDrive(), startPoseInchRad, builder.getBaseVelConstraint(), builder.getBaseAccelConstraint(), builder.getBaseTurnConstraintMaxAngVel(), builder.getBaseTurnConstraintMaxAngAccel());
    }


    /**
     * Use this method to build a new RoadRunner trajectory or to add a RoadRunner trajectory to the task queue.
     *
     * @param startPose Starting pose of the trajectory, <b>WILL SET DRIVE POSE ESTIMATE TO THIS POSE</b>
     * @param inUnit    The unit of the end pose vector (will be converted to inches)
     * @param angleUnit The unit of the end pose heading (will be converted to radians)
     * @return Builder for the trajectory
     */
    default RoadRunnerTrajectoryTaskBuilder makeTrajectory(Pose2d startPose, Distance inUnit, Angle angleUnit) {
        double x = Inches.convertFrom(startPose.getX(), inUnit);
        double y = Inches.convertFrom(startPose.getY(), inUnit);
        double r = Radians.convertFrom(startPose.getHeading(), angleUnit);
        return makeTrajectory(new Pose2d(x, y, r));
    }

    /**
     * Use this method to build a new RoadRunner trajectory or to add a RoadRunner trajectory to the task queue.
     * Without arguments, will use the current pose estimate of the drive or the last spliced pose.
     *
     * @return Builder for the trajectory
     * @see #makeTrajectory(Pose2d)
     */
    default RoadRunnerTrajectoryTaskBuilder makeTrajectory() {
        // noinspection rawtypes
        TrajectorySequenceBuilder builder = getDrive().trajectorySequenceBuilder(getPreviousPose());
        return new RoadRunnerTrajectoryTaskBuilder(getDrive(), getPreviousPose(), builder.getBaseVelConstraint(), builder.getBaseAccelConstraint(), builder.getBaseTurnConstraintMaxAngVel(), builder.getBaseTurnConstraintMaxAngAccel());
    }

    /**
     * Priority representation for building tasks.
     */
    enum PriorityLevel {
        /**
         * Add the task to the end of the queue after the onReady() init callback has fired
         */
        LAST,
        /**
         * Add the task to the queue immediately (default)
         */
        NORMAL,
        /**
         * Add the task to the front of the queue after the onReady() init callback has fired
         */
        FIRST//® Tech Challenge
    }

    /**
     * Builder class for a trajectory and task sequence.
     * This is a more advanced builder for creating a sequence of trajectories and tasks, allowing
     * task queues and adding trajectory sequences standalone.
     */
    final class RoadRunnerTrajectoryTaskBuilder extends TrajectorySequenceBuilder<RoadRunnerTrajectoryTaskBuilder> {
        private final TrajectorySequenceBuilder<RoadRunnerTrajectoryTaskBuilder> mirroredBuilder;
        private final RoadRunnerDrive drive;
        private TrajectorySequence overrideSequence;
        private Measure<Time> timeout = DEFAULT_TIMEOUT;
        private PriorityLevel priority = PriorityLevel.NORMAL;
        private Reference<TrajectorySequence> mirroredTrajectory = null;
        private String name = null;

        /**
         * Create a new builder for a RoadRunner trajectory task.
         *
         * @param drive                         Drive system to use
         * @param startPose                     Starting pose of the trajectory (in, in, radians)
         * @param startTangent                  Starting tangent of the trajectory (radians)
         * @param baseVelConstraint             Base velocity constraint (in/s)
         * @param baseAccelConstraint           Base acceleration constraint (in/s^2)
         * @param baseTurnConstraintMaxAngVel   Base turn constraint max angular velocity (rad/s)
         * @param baseTurnConstraintMaxAngAccel Base turn constraint max angular acceleration (rad/s^2)
         */
        public RoadRunnerTrajectoryTaskBuilder(RoadRunnerDrive drive, Pose2d startPose, Double startTangent, TrajectoryVelocityConstraint baseVelConstraint, TrajectoryAccelerationConstraint baseAccelConstraint, double baseTurnConstraintMaxAngVel, double baseTurnConstraintMaxAngAccel) {
            super(startPose, startTangent, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
            mirroredBuilder = new TrajectorySequenceBuilder<>(new Pose2d(startPose.getX(), -startPose.getY(), Mathf.normaliseAngle(Radians.of(startPose.getHeading()).plus(Radians.of(Math.PI))).in(Radians)), startTangent, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
            this.drive = drive;
        }

        /**
         * Create a new builder for a RoadRunner trajectory task.
         *
         * @param drive                         Drive system to use
         * @param startPose                     Starting pose of the trajectory (in, in, radians)
         * @param baseVelConstraint             Base velocity constraint
         * @param baseAccelConstraint           Base acceleration constraint
         * @param baseTurnConstraintMaxAngVel   Base turn constraint max angular velocity
         * @param baseTurnConstraintMaxAngAccel Base turn constraint max angular acceleration
         */
        public RoadRunnerTrajectoryTaskBuilder(RoadRunnerDrive drive, Pose2d startPose, TrajectoryVelocityConstraint baseVelConstraint, TrajectoryAccelerationConstraint baseAccelConstraint, double baseTurnConstraintMaxAngVel, double baseTurnConstraintMaxAngAccel) {
            super(startPose, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
            mirroredBuilder = new TrajectorySequenceBuilder<>(new Pose2d(startPose.getX(), -startPose.getY(), Mathf.normaliseAngle(Radians.of(startPose.getHeading()).plus(Radians.of(Math.PI))).in(Radians)), baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
            this.drive = drive;
        }

        /**
         * Move in a straight line to a given position.
         *
         * @param endPositionInches The end position (inches)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineTo(Vector2d endPositionInches) {
            mirroredBuilder.lineTo(new Vector2d(endPositionInches.getX(), -endPositionInches.getY()));
            return super.lineTo(endPositionInches);
        }

        /**
         * Move in a straight line to a given position.
         *
         * @param endPosition The end position
         * @param inUnit      The unit of the end position vector (will be converted to inches)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineTo(Vector2d endPosition, Distance inUnit) {
            mirroredBuilder.lineTo(new Vector2d(endPosition.getX(), -endPosition.getY()), inUnit);
            double x = Inches.convertFrom(endPosition.getX(), inUnit);
            double y = Inches.convertFrom(endPosition.getY(), inUnit);
            return lineTo(new Vector2d(x, y));
        }

        /**
         * Move in a straight line to a given position with custom velocity and acceleration constraints.
         *
         * @param endPositionInches The end position (inches)
         * @param velConstraint     The velocity constraint (inches/sec)
         * @param accelConstraint   The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineTo(
                Vector2d endPositionInches,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.lineTo(new Vector2d(endPositionInches.getX(), -endPositionInches.getY()), velConstraint, accelConstraint);
            return super.lineTo(endPositionInches, velConstraint, accelConstraint);
        }

        /**
         * Move in a straight line to a given position with custom velocity and acceleration constraints.
         *
         * @param endPositionInches The end position (inches)
         * @param inUnit            The unit of the end position vector (will be converted to inches)
         * @param velConstraint     The velocity constraint (inches/sec)
         * @param accelConstraint   The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineTo(
                Vector2d endPositionInches,
                Distance inUnit,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.lineTo(new Vector2d(endPositionInches.getX(), -endPositionInches.getY()), inUnit, velConstraint, accelConstraint);
            return super.lineTo(endPositionInches, inUnit, velConstraint, accelConstraint);
        }

        /**
         * Move in a straight line to a given position with a constant heading.
         *
         * @param endPositionInches The end position (inches)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineToConstantHeading(Vector2d endPositionInches) {
            mirroredBuilder.lineToConstantHeading(new Vector2d(endPositionInches.getX(), -endPositionInches.getY()));
            return super.lineToConstantHeading(endPositionInches);
        }

        /**
         * Move in a straight line to a given position with a constant heading.
         *
         * @param endPosition The end position
         * @param inUnit      The unit of the end position vector (will be converted to inches)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineToConstantHeading(Vector2d endPosition, Distance inUnit) {
            mirroredBuilder.lineToConstantHeading(new Vector2d(endPosition.getX(), -endPosition.getY()), inUnit);
            return super.lineToConstantHeading(endPosition, inUnit);
        }

        /**
         * Move in a straight line to a given position with a constant heading and custom velocity and acceleration constraints.
         *
         * @param endPositionInches The end position (inches)
         * @param velConstraint     The velocity constraint (inches/sec)
         * @param accelConstraint   The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineToConstantHeading(
                Vector2d endPositionInches,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.lineToConstantHeading(new Vector2d(endPositionInches.getX(), -endPositionInches.getY()), velConstraint, accelConstraint);
            return super.lineToConstantHeading(endPositionInches, velConstraint, accelConstraint);
        }

        /**
         * Move in a straight line to a given position with a constant heading and custom velocity and acceleration constraints.
         *
         * @param endPosition     The end position
         * @param inUnit          The unit of the end position vector (will be converted to inches)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineToConstantHeading(
                Vector2d endPosition,
                Distance inUnit,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.lineToConstantHeading(new Vector2d(endPosition.getX(), -endPosition.getY()), inUnit, velConstraint, accelConstraint);
            return super.lineToConstantHeading(endPosition, inUnit, velConstraint, accelConstraint);
        }

        /**
         * Move in a straight line to a given position with a linear heading.
         *
         * @param endPoseInchRad The end pose (in, in, radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineToLinearHeading(Pose2d endPoseInchRad) {
            mirroredBuilder.lineToLinearHeading(new Pose2d(endPoseInchRad.getX(), -endPoseInchRad.getY(), Mathf.normaliseAngle(Radians.of(endPoseInchRad.getHeading()).plus(Radians.of(Math.PI))).in(Radians)));
            return super.lineToLinearHeading(endPoseInchRad);
        }

        /**
         * Move in a straight line to a given position with a linear heading.
         *
         * @param endPose      The end pose
         * @param distanceUnit The unit of the end pose vector (will be converted to inches)
         * @param angleUnit    The unit of the end pose heading (will be converted to radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineToLinearHeading(Pose2d endPose, Distance distanceUnit, Angle angleUnit) {
            mirroredBuilder.lineToLinearHeading(new Pose2d(endPose.getX(), -endPose.getY(), Mathf.normaliseAngle(angleUnit.of(endPose.getHeading()).plus(Radians.of(Math.PI))).in(angleUnit)), distanceUnit, angleUnit);
            return super.lineToLinearHeading(endPose, distanceUnit, angleUnit);
        }

        /**
         * Move in a straight line to a given position with a linear heading and custom velocity and acceleration constraints.
         *
         * @param endPoseInchRad  The end pose (in, in, radians)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineToLinearHeading(
                Pose2d endPoseInchRad,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.lineToLinearHeading(new Pose2d(endPoseInchRad.getX(), -endPoseInchRad.getY(), Mathf.normaliseAngle(Radians.of(endPoseInchRad.getHeading()).plus(Radians.of(Math.PI))).in(Radians)), velConstraint, accelConstraint);
            return super.lineToLinearHeading(endPoseInchRad, velConstraint, accelConstraint);
        }

        /**
         * Move in a straight line to a given position with a linear heading and custom velocity and acceleration constraints.
         *
         * @param endPose         The end pose
         * @param distanceUnit    The unit of the end pose vector (will be converted to inches)
         * @param angleUnit       The unit of the end pose heading (will be converted to radians)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineToLinearHeading(
                Pose2d endPose,
                Distance distanceUnit,
                Angle angleUnit,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.lineToLinearHeading(new Pose2d(endPose.getX(), -endPose.getY(), Mathf.normaliseAngle(angleUnit.of(endPose.getHeading()).plus(Radians.of(Math.PI))).in(angleUnit)), distanceUnit, angleUnit, velConstraint, accelConstraint);
            return super.lineToLinearHeading(endPose, distanceUnit, angleUnit, velConstraint, accelConstraint);
        }

        /**
         * Move in a straight line to a given position with a spline heading.
         *
         * @param endPoseInchRad The end pose (in, in, radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineToSplineHeading(Pose2d endPoseInchRad) {
            mirroredBuilder.lineToSplineHeading(new Pose2d(endPoseInchRad.getX(), -endPoseInchRad.getY(), Mathf.normaliseAngle(Radians.of(endPoseInchRad.getHeading()).plus(Radians.of(Math.PI))).in(Radians)));
            return super.lineToSplineHeading(endPoseInchRad);
        }

        /**
         * Move in a straight line to a given position with a spline heading.
         *
         * @param endPose      The end pose
         * @param distanceUnit The unit of the end pose vector (will be converted to inches)
         * @param angleUnit    The unit of the end pose heading (will be converted to radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineToSplineHeading(Pose2d endPose, Distance distanceUnit, Angle angleUnit) {
            mirroredBuilder.lineToSplineHeading(new Pose2d(endPose.getX(), -endPose.getY(), Mathf.normaliseAngle(angleUnit.of(endPose.getHeading()).plus(Radians.of(Math.PI))).in(angleUnit)), distanceUnit, angleUnit);
            return super.lineToSplineHeading(endPose, distanceUnit, angleUnit);
        }

        /**
         * Move in a straight line to a given position with a spline heading and custom velocity and acceleration constraints.
         *
         * @param endPoseInchRad  The end pose (in, in, radians)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineToSplineHeading(
                Pose2d endPoseInchRad,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.lineToSplineHeading(new Pose2d(endPoseInchRad.getX(), -endPoseInchRad.getY(), Mathf.normaliseAngle(Radians.of(endPoseInchRad.getHeading()).plus(Radians.of(Math.PI))).in(Radians)), velConstraint, accelConstraint);
            return super.lineToSplineHeading(endPoseInchRad, velConstraint, accelConstraint);
        }

        /**
         * Move in a straight line to a given position with a spline heading and custom velocity and acceleration constraints.
         *
         * @param endPose         The end pose
         * @param distanceUnit    The unit of the end pose vector (will be converted to inches)
         * @param angleUnit       The unit of the end pose heading (will be converted to radians)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder lineToSplineHeading(
                Pose2d endPose,
                Distance distanceUnit,
                Angle angleUnit,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.lineToSplineHeading(new Pose2d(endPose.getX(), -endPose.getY(), Mathf.normaliseAngle(angleUnit.of(endPose.getHeading()).plus(Radians.of(Math.PI))).in(angleUnit)), distanceUnit, angleUnit, velConstraint, accelConstraint);
            return super.lineToSplineHeading(endPose, distanceUnit, angleUnit, velConstraint, accelConstraint);
        }

        /**
         * Move in a strafe straight line to a given position.
         *
         * @param endPositionInches The end position (inches)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder strafeTo(Vector2d endPositionInches) {
            mirroredBuilder.strafeTo(new Vector2d(endPositionInches.getX(), -endPositionInches.getY()));
            return super.strafeTo(endPositionInches);
        }

        /**
         * Move in a strafe straight line to a given position.
         *
         * @param endPosition The end position
         * @param inUnit      The unit of the end position vector (will be converted to inches)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder strafeTo(Vector2d endPosition, Distance inUnit) {
            mirroredBuilder.strafeTo(new Vector2d(endPosition.getX(), -endPosition.getY()), inUnit);
            return super.strafeTo(endPosition, inUnit);
        }

        /**
         * Move in a strafe straight line to a given position with custom velocity and acceleration constraints.
         *
         * @param endPositionInches The end position (inches)
         * @param velConstraint     The velocity constraint (inches/sec)
         * @param accelConstraint   The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder strafeTo(
                Vector2d endPositionInches,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.strafeTo(new Vector2d(endPositionInches.getX(), -endPositionInches.getY()), velConstraint, accelConstraint);
            return super.strafeTo(endPositionInches, velConstraint, accelConstraint);
        }

        /**
         * Move in a strafe straight line to a given position with custom velocity and acceleration constraints.
         *
         * @param endPosition     The end position
         * @param inUnit          The unit of the end position vector (will be converted to inches)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder strafeTo(
                Vector2d endPosition,
                Distance inUnit,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.strafeTo(new Vector2d(endPosition.getX(), -endPosition.getY()), inUnit, velConstraint, accelConstraint);
            return super.strafeTo(endPosition, inUnit, velConstraint, accelConstraint);
        }

        /**
         * Move forward a given distance.
         *
         * @param inches The distance to move (inches)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder forward(double inches) {
            mirroredBuilder.forward(inches);
            return super.forward(inches);
        }

        /**
         * Move forward a given distance.
         *
         * @param distance The distance to move
         * @param inUnit   The unit of the distance (will be converted to inches)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder forward(double distance, Distance inUnit) {
            mirroredBuilder.forward(distance, inUnit);
            return super.forward(distance, inUnit);
        }

        /**
         * Move forward a given distance with custom velocity and acceleration constraints.
         *
         * @param inches          The distance to move (inches)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder forward(
                double inches,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.forward(inches, velConstraint, accelConstraint);
            return super.forward(inches, velConstraint, accelConstraint);
        }

        /**
         * Move forward a given distance with custom velocity and acceleration constraints.
         *
         * @param distance        The distance to move
         * @param inUnit          The unit of the distance (will be converted to inches)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder forward(
                double distance,
                Distance inUnit,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.forward(distance, inUnit, velConstraint, accelConstraint);
            return super.forward(distance, inUnit, velConstraint, accelConstraint);
        }

        /**
         * Move backward a given distance.
         *
         * @param inches The distance to move (inches)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder back(double inches) {
            mirroredBuilder.back(inches);
            return super.back(inches);
        }

        /**
         * Move backward a given distance.
         *
         * @param distance The distance to move
         * @param inUnit   The unit of the distance (will be converted to inches)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder back(double distance, Distance inUnit) {
            mirroredBuilder.back(distance, inUnit);
            return super.back(distance, inUnit);
        }

        /**
         * Move backward a given distance with custom velocity and acceleration constraints.
         *
         * @param inches          The distance to move (inches)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder back(
                double inches,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.back(inches, velConstraint, accelConstraint);
            return super.back(inches, velConstraint, accelConstraint);
        }

        /**
         * Move backward a given distance with custom velocity and acceleration constraints.
         *
         * @param distance        The distance to move
         * @param inUnit          The unit of the distance (will be converted to inches)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder back(
                double distance,
                Distance inUnit,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.back(distance, inUnit, velConstraint, accelConstraint);
            return super.back(distance, inUnit, velConstraint, accelConstraint);
        }

        /**
         * Strafe left a given distance.
         *
         * @param inches The distance to strafe (inches)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder strafeLeft(double inches) {
            mirroredBuilder.strafeRight(inches);
            return super.strafeLeft(inches);
        }

        /**
         * Strafe left a given distance.
         *
         * @param distance The distance to strafe
         * @param inUnit   The unit of the distance (will be converted to inches)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder strafeLeft(double distance, Distance inUnit) {
            mirroredBuilder.strafeRight(distance, inUnit);
            return super.strafeLeft(distance, inUnit);
        }

        /**
         * Strafe left a given distance with custom velocity and acceleration constraints.
         *
         * @param inches          The distance to strafe (inches)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder strafeLeft(
                double inches,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.strafeRight(inches, velConstraint, accelConstraint);
            return super.strafeLeft(inches, velConstraint, accelConstraint);
        }

        /**
         * Strafe left a given distance with custom velocity and acceleration constraints.
         *
         * @param distance        The distance to strafe
         * @param inUnit          The unit of the distance (will be converted to inches)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder strafeLeft(
                double distance,
                Distance inUnit,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.strafeRight(distance, inUnit, velConstraint, accelConstraint);
            return super.strafeLeft(distance, inUnit, velConstraint, accelConstraint);
        }

        /**
         * Strafe right a given distance.
         *
         * @param inches The distance to strafe (inches)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder strafeRight(double inches) {
            mirroredBuilder.strafeLeft(inches);
            return super.strafeRight(inches);
        }

        /**
         * Strafe right a given distance.
         *
         * @param distance The distance to strafe
         * @param inUnit   The unit of the distance (will be converted to inches)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder strafeRight(double distance, Distance inUnit) {
            mirroredBuilder.strafeLeft(distance, inUnit);
            return super.strafeRight(distance, inUnit);
        }

        /**
         * Strafe right a given distance with custom velocity and acceleration constraints.
         *
         * @param inches          The distance to strafe (inches)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder strafeRight(
                double inches,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.strafeLeft(inches, velConstraint, accelConstraint);
            return super.strafeRight(inches, velConstraint, accelConstraint);
        }

        /**
         * Strafe right a given distance with custom velocity and acceleration constraints.
         *
         * @param distance        The distance to strafe
         * @param inUnit          The unit of the distance (will be converted to inches)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder strafeRight(
                double distance,
                Distance inUnit,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.strafeLeft(distance, inUnit, velConstraint, accelConstraint);
            return super.strafeRight(distance, inUnit, velConstraint, accelConstraint);
        }

        /**
         * Spline to a given position with a given heading.
         *
         * @param endPositionInches The end position (inches)
         * @param endHeadingRad     The end heading (radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineTo(Vector2d endPositionInches, double endHeadingRad) {
            mirroredBuilder.splineTo(new Vector2d(endPositionInches.getX(), -endPositionInches.getY()), Mathf.normaliseAngle(Radians.of(endHeadingRad).plus(Radians.of(Math.PI))).in(Radians));
            return super.splineTo(endPositionInches, endHeadingRad);
        }

        /**
         * Spline to a given position with a given heading.
         *
         * @param endPosition The end position
         * @param inUnit      The unit of the end position vector (will be converted to inches)
         * @param endHeading  The end heading
         * @param angleUnit   The unit of the end heading (will be converted to radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineTo(Vector2d endPosition, Distance inUnit, double endHeading, Angle angleUnit) {
            mirroredBuilder.splineTo(new Vector2d(endPosition.getX(), -endPosition.getY()), inUnit, Mathf.normaliseAngle(angleUnit.of(endHeading).plus(Radians.of(Math.PI))).in(angleUnit), angleUnit);
            return super.splineTo(endPosition, inUnit, endHeading, angleUnit);
        }

        /**
         * Spline to a given position with a given heading and custom velocity and acceleration constraints.
         *
         * @param endPositionInches The end position (inches)
         * @param endHeadingRad     The end heading (radians)
         * @param velConstraint     The velocity constraint (inches/sec)
         * @param accelConstraint   The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineTo(
                Vector2d endPositionInches,
                double endHeadingRad,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.splineTo(new Vector2d(endPositionInches.getX(), -endPositionInches.getY()), Mathf.normaliseAngle(Radians.of(endHeadingRad).plus(Radians.of(Math.PI))).in(Radians), velConstraint, accelConstraint);
            return super.splineTo(endPositionInches, endHeadingRad, velConstraint, accelConstraint);
        }

        /**
         * Spline to a given position with a given heading and custom velocity and acceleration constraints.
         *
         * @param endPosition     The end position
         * @param inUnit          The unit of the end position vector (will be converted to inches)
         * @param endHeading      The end heading
         * @param angleUnit       The unit of the end heading (will be converted to radians)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineTo(
                Vector2d endPosition,
                Distance inUnit,
                double endHeading,
                Angle angleUnit,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.splineTo(new Vector2d(endPosition.getX(), -endPosition.getY()), inUnit, Mathf.normaliseAngle(angleUnit.of(endHeading).plus(Radians.of(Math.PI))).in(angleUnit), angleUnit, velConstraint, accelConstraint);
            return super.splineTo(endPosition, inUnit, endHeading, angleUnit, velConstraint, accelConstraint);
        }

        /**
         * Spline to a given position with a constant heading.
         *
         * @param endPositionInches The end position (inches)
         * @param endHeadingRad     The end heading (radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineToConstantHeading(Vector2d endPositionInches, double endHeadingRad) {
            mirroredBuilder.splineToConstantHeading(new Vector2d(endPositionInches.getX(), -endPositionInches.getY()), Mathf.normaliseAngle(Radians.of(endHeadingRad).plus(Radians.of(Math.PI))).in(Radians));
            return super.splineToConstantHeading(endPositionInches, endHeadingRad);
        }

        /**
         * Spline to a given position with a constant heading.
         *
         * @param endPosition The end position
         * @param inUnit      The unit of the end position vector (will be converted to inches)
         * @param endHeading  The end heading
         * @param angleUnit   The unit of the end heading (will be converted to radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineToConstantHeading(Vector2d endPosition, Distance inUnit, double endHeading, Angle angleUnit) {
            mirroredBuilder.splineToConstantHeading(new Vector2d(endPosition.getX(), -endPosition.getY()), inUnit, Mathf.normaliseAngle(angleUnit.of(endHeading).plus(Radians.of(Math.PI))).in(angleUnit), angleUnit);
            return super.splineToConstantHeading(endPosition, inUnit, endHeading, angleUnit);
        }

        /**
         * Spline to a given position with a constant heading and custom velocity and acceleration constraints.
         *
         * @param endPositionInches The end position (inches)
         * @param endHeadingRad     The end heading (radians)
         * @param velConstraint     The velocity constraint (inches/sec)
         * @param accelConstraint   The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineToConstantHeading(
                Vector2d endPositionInches,
                double endHeadingRad,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.splineToConstantHeading(new Vector2d(endPositionInches.getX(), -endPositionInches.getY()), Mathf.normaliseAngle(Radians.of(endHeadingRad).plus(Radians.of(Math.PI))).in(Radians), velConstraint, accelConstraint);
            return super.splineToConstantHeading(endPositionInches, endHeadingRad, velConstraint, accelConstraint);
        }

        /**
         * Spline to a given position with a constant heading and custom velocity and acceleration constraints.
         *
         * @param endPosition     The end position
         * @param inUnit          The unit of the end position vector (will be converted to inches)
         * @param endHeading      The end heading
         * @param angleUnit       The unit of the end heading (will be converted to radians)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineToConstantHeading(
                Vector2d endPosition,
                Distance inUnit,
                double endHeading,
                Angle angleUnit,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.splineToConstantHeading(new Vector2d(endPosition.getX(), -endPosition.getY()), inUnit, Mathf.normaliseAngle(angleUnit.of(endHeading).plus(Radians.of(Math.PI))).in(angleUnit), angleUnit, velConstraint, accelConstraint);
            return super.splineToConstantHeading(endPosition, inUnit, endHeading, angleUnit, velConstraint, accelConstraint);
        }

        /**
         * Spline to a given position with a linear heading.
         *
         * @param endPoseInchRad The end pose (in, in, radians)
         * @param endHeadingRad  The end heading (radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineToLinearHeading(Pose2d endPoseInchRad, double endHeadingRad) {
            mirroredBuilder.splineToLinearHeading(new Pose2d(endPoseInchRad.getX(), -endPoseInchRad.getY(), Mathf.normaliseAngle(Radians.of(endPoseInchRad.getHeading()).plus(Radians.of(Math.PI))).in(Radians)), endHeadingRad);
            return super.splineToLinearHeading(endPoseInchRad, endHeadingRad);
        }

        /**
         * Spline to a given position with a linear heading.
         *
         * @param endPose      The end pose
         * @param distanceUnit The unit of the end pose vector (will be converted to inches)
         * @param angleUnit    The unit of the end pose heading (will be converted to radians)
         * @param endHeading   The end heading
         * @param endAngleUnit The unit of the end heading (will be converted to radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineToLinearHeading(Pose2d endPose, Distance distanceUnit, Angle angleUnit, double endHeading, Angle endAngleUnit) {
            mirroredBuilder.splineToLinearHeading(new Pose2d(endPose.getX(), -endPose.getY(), Mathf.normaliseAngle(angleUnit.of(endPose.getHeading()).plus(Radians.of(Math.PI))).in(angleUnit)), distanceUnit, angleUnit, endHeading, endAngleUnit);
            return super.splineToLinearHeading(endPose, distanceUnit, angleUnit, endHeading, endAngleUnit);
        }

        /**
         * Spline to a given position with a linear heading and custom velocity and acceleration constraints.
         *
         * @param endPoseInchRad  The end pose (in, in, radians)
         * @param endHeadingRad   The end heading (radians)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineToLinearHeading(
                Pose2d endPoseInchRad,
                double endHeadingRad,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.splineToLinearHeading(new Pose2d(endPoseInchRad.getX(), -endPoseInchRad.getY(), Mathf.normaliseAngle(Radians.of(endPoseInchRad.getHeading()).plus(Radians.of(Math.PI))).in(Radians)), Mathf.normaliseAngle(Radians.of(endHeadingRad).plus(Radians.of(Math.PI))).in(Radians), velConstraint, accelConstraint);
            return super.splineToLinearHeading(endPoseInchRad, endHeadingRad, velConstraint, accelConstraint);
        }

        /**
         * Spline to a given position with a linear heading and custom velocity and acceleration constraints.
         *
         * @param endPose         The end pose
         * @param distanceUnit    The unit of the end pose vector (will be converted to inches)
         * @param angleUnit       The unit of the end pose heading (will be converted to radians)
         * @param endHeading      The end heading
         * @param endAngleUnit    The unit of the end heading (will be converted to radians)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineToLinearHeading(
                Pose2d endPose,
                Distance distanceUnit,
                Angle angleUnit,
                double endHeading,
                Angle endAngleUnit,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.splineToLinearHeading(new Pose2d(endPose.getX(), -endPose.getY(), Mathf.normaliseAngle(angleUnit.of(endPose.getHeading()).plus(Radians.of(Math.PI))).in(angleUnit)), distanceUnit, angleUnit, Mathf.normaliseAngle(endAngleUnit.of(endHeading).plus(Radians.of(Math.PI))).in(endAngleUnit), endAngleUnit, velConstraint, accelConstraint);
            double x = Inches.convertFrom(endPose.getX(), distanceUnit);
            double y = Inches.convertFrom(endPose.getY(), distanceUnit);
            double r = Radians.convertFrom(endHeading, angleUnit);
            double r2 = Radians.convertFrom(endPose.getHeading(), endAngleUnit);
            return splineToLinearHeading(new Pose2d(x, y, r), r2, velConstraint, accelConstraint);
        }

        /**
         * Spline to a given position with a spline heading.
         *
         * @param endPoseInchRad The end pose (in, in, radians)
         * @param endHeadingRad  The end heading (radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineToSplineHeading(Pose2d endPoseInchRad, double endHeadingRad) {
            mirroredBuilder.splineToSplineHeading(new Pose2d(endPoseInchRad.getX(), -endPoseInchRad.getY(), Mathf.normaliseAngle(Radians.of(endPoseInchRad.getHeading()).plus(Radians.of(Math.PI))).in(Radians)), Mathf.normaliseAngle(Radians.of(endPoseInchRad.getHeading()).plus(Radians.of(Math.PI))).in(Radians));
            return super.splineToSplineHeading(endPoseInchRad, endHeadingRad);
        }

        /**
         * Spline to a given position with a spline heading.
         *
         * @param endPose      The end pose
         * @param distanceUnit The unit of the end pose vector (will be converted to inches)
         * @param angleUnit    The unit of the end pose heading (will be converted to radians)
         * @param endHeading   The end heading
         * @param endAngleUnit The unit of the end heading (will be converted to radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineToSplineHeading(Pose2d endPose, Distance distanceUnit, Angle angleUnit, double endHeading, Angle endAngleUnit) {
            mirroredBuilder.splineToSplineHeading(new Pose2d(endPose.getX(), -endPose.getY(), Mathf.normaliseAngle(angleUnit.of(endPose.getHeading()).plus(Radians.of(Math.PI))).in(angleUnit)), distanceUnit, angleUnit, Mathf.normaliseAngle(endAngleUnit.of(endPose.getHeading()).plus(Radians.of(Math.PI))).in(endAngleUnit), endAngleUnit);
            return super.splineToSplineHeading(endPose, distanceUnit, angleUnit, endHeading, endAngleUnit);
        }

        /**
         * Spline to a given position with a spline heading and custom velocity and acceleration constraints.
         *
         * @param endPoseInchRad  The end pose (in, in, radians)
         * @param endHeadingRad   The end heading (radians)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineToSplineHeading(
                Pose2d endPoseInchRad,
                double endHeadingRad,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.splineToSplineHeading(new Pose2d(endPoseInchRad.getX(), -endPoseInchRad.getY(), Mathf.normaliseAngle(Radians.of(endPoseInchRad.getHeading()).plus(Radians.of(Math.PI))).in(Radians)), Mathf.normaliseAngle(Radians.of(endPoseInchRad.getHeading()).plus(Radians.of(Math.PI))).in(Radians), velConstraint, accelConstraint);
            return super.splineToSplineHeading(endPoseInchRad, endHeadingRad, velConstraint, accelConstraint);
        }

        /**
         * Spline to a given position with a spline heading and custom velocity and acceleration constraints.
         *
         * @param endPose         The end pose
         * @param distanceUnit    The unit of the end pose vector (will be converted to inches)
         * @param angleUnit       The unit of the end pose heading (will be converted to radians)
         * @param endHeading      The end heading
         * @param endAngleUnit    The unit of the end heading (will be converted to radians)
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder splineToSplineHeading(
                Pose2d endPose,
                Distance distanceUnit,
                Angle angleUnit,
                double endHeading,
                Angle endAngleUnit,
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.splineToSplineHeading(new Pose2d(endPose.getX(), -endPose.getY(), Mathf.normaliseAngle(angleUnit.of(endPose.getHeading()).plus(Radians.of(Math.PI))).in(angleUnit)), distanceUnit, angleUnit, Mathf.normaliseAngle(endAngleUnit.of(endPose.getHeading()).plus(Radians.of(Math.PI))).in(endAngleUnit), endAngleUnit, velConstraint, accelConstraint);
            return super.splineToSplineHeading(endPose, distanceUnit, angleUnit, endHeading, endAngleUnit, velConstraint, accelConstraint);
        }

        /**
         * Set the tangent of the next path.
         *
         * @param tangent The tangent (radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder setTangent(double tangent) {
            mirroredBuilder.setTangent(tangent);
            return super.setTangent(tangent);
        }

        /**
         * Set the tangent of the next path.
         *
         * @param tangent   The tangent
         * @param angleUnit The unit of the tangent (will be converted to radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder setTangent(double tangent, Angle angleUnit) {
            mirroredBuilder.setTangent(tangent, angleUnit);
            return super.setTangent(tangent, angleUnit);
        }

        /**
         * Reverse or unreverse the tangent of the next path.
         *
         * @param reversed Whether to reverse the tangent
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder setReversed(boolean reversed) {
            mirroredBuilder.setReversed(reversed);
            return super.setReversed(reversed);
        }

        /**
         * Set the velocity and acceleration constraints for the next builder instructions.
         *
         * @param velConstraint   The velocity constraint (inches/sec)
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder setConstraints(
                TrajectoryVelocityConstraint velConstraint,
                TrajectoryAccelerationConstraint accelConstraint
        ) {
            mirroredBuilder.setConstraints(velConstraint, accelConstraint);
            return super.setConstraints(velConstraint, accelConstraint);
        }

        /**
         * Reset the velocity and acceleration constraints to the base constraints.
         *
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder resetConstraints() {
            mirroredBuilder.resetConstraints();
            return super.resetConstraints();
        }

        /**
         * Set the velocity constraint for the next builder instructions.
         *
         * @param velConstraint The velocity constraint (inches/sec)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder setVelConstraint(TrajectoryVelocityConstraint velConstraint) {
            mirroredBuilder.setVelConstraint(velConstraint);
            return super.setVelConstraint(velConstraint);
        }

        /**
         * Reset the velocity constraint to the base velocity constraint.
         *
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder resetVelConstraint() {
            mirroredBuilder.resetVelConstraint();
            return super.resetVelConstraint();
        }

        /**
         * Set the acceleration constraint for the next builder instructions.
         *
         * @param accelConstraint The acceleration constraint (inches/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder setAccelConstraint(TrajectoryAccelerationConstraint accelConstraint) {
            mirroredBuilder.setAccelConstraint(accelConstraint);
            return super.setAccelConstraint(accelConstraint);
        }

        /**
         * Reset the acceleration constraint to the base acceleration constraint.
         *
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder resetAccelConstraint() {
            mirroredBuilder.resetAccelConstraint();
            return super.resetAccelConstraint();
        }

        /**
         * Set the turn constraints for the next builder instructions.
         *
         * @param maxAngVel   The maximum angular velocity (radians/sec)
         * @param maxAngAccel The maximum angular acceleration (radians/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder setTurnConstraint(double maxAngVel, double maxAngAccel) {
            mirroredBuilder.setTurnConstraint(maxAngVel, maxAngAccel);
            return super.setTurnConstraint(maxAngVel, maxAngAccel);
        }

        /**
         * Set the turn constraints for the next builder instructions.
         *
         * @param maxAngVel   The maximum angular velocity
         * @param velUnit     The unit of the maximum angular velocity (will be converted to radians/sec)
         * @param maxAngAccel The maximum angular acceleration
         * @param accelUnit   The unit of the maximum angular acceleration (will be converted to radians/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder setTurnConstraint(double maxAngVel, Velocity<Angle> velUnit, double maxAngAccel, Velocity<Velocity<Angle>> accelUnit) {
            mirroredBuilder.setTurnConstraint(maxAngVel, velUnit, maxAngAccel, accelUnit);
            return super.setTurnConstraint(maxAngVel, velUnit, maxAngAccel, accelUnit);
        }

        /**
         * Reset the turn constraints to the base turn constraints.
         *
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder resetTurnConstraint() {
            mirroredBuilder.resetTurnConstraint();
            return super.resetTurnConstraint();
        }

        /**
         * Add a temporal marker at the current duration to run a callback at that time.
         *
         * @param callback The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addTemporalMarker(MarkerCallback callback) {
            mirroredBuilder.addTemporalMarker(callback);
            return super.addTemporalMarker(callback);
        }

        /**
         * Add a temporal marker at the current duration plus an offset to run a callback at that time.
         *
         * @param offset   The offset to add to the current duration (seconds)
         * @param callback The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder UNSTABLE_addTemporalMarkerOffset(double offset, MarkerCallback callback) {
            mirroredBuilder.UNSTABLE_addTemporalMarkerOffset(offset, callback);
            return super.UNSTABLE_addTemporalMarkerOffset(offset, callback);
        }

        /**
         * Add a temporal marker at the current duration plus an offset to run a callback at that time.
         *
         * @param offset   The offset to add to the current duration
         * @param callback The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder UNSTABLE_addTemporalMarkerOffset(Measure<Time> offset, MarkerCallback callback) {
            mirroredBuilder.UNSTABLE_addTemporalMarkerOffset(offset, callback);
            return super.UNSTABLE_addTemporalMarkerOffset(offset, callback);
        }

        /**
         * Add a temporal marker at a given time to run a callback at that time.
         *
         * @param time     The time to run the callback (seconds)
         * @param callback The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addTemporalMarker(double time, MarkerCallback callback) {
            mirroredBuilder.addTemporalMarker(time, callback);
            return super.addTemporalMarker(time, callback);
        }

        /**
         * Add a temporal marker at a given time to run a callback at that time.
         *
         * @param time     The time to run the callback
         * @param callback The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addTemporalMarker(Measure<Time> time, MarkerCallback callback) {
            mirroredBuilder.addTemporalMarker(time, callback);
            return super.addTemporalMarker(time, callback);
        }

        /**
         * Add a temporal marker at a given time to run a callback at that time.
         *
         * @param scale    A multiplicative scale to apply to the time
         * @param offset   The offset to add to the time (seconds)
         * @param callback The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addTemporalMarker(double scale, double offset, MarkerCallback callback) {
            mirroredBuilder.addTemporalMarker(scale, offset, callback);
            return super.addTemporalMarker(scale, offset, callback);
        }

        /**
         * Add a temporal marker at a given time to run a callback at that time.
         *
         * @param scale    A multiplicative scale to apply to the time
         * @param offset   The offset to add to the time
         * @param callback The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addTemporalMarker(double scale, Measure<Time> offset, MarkerCallback callback) {
            mirroredBuilder.addTemporalMarker(scale, offset, callback);
            return super.addTemporalMarker(scale, offset, callback);
        }

        /**
         * Add a temporal marker at a given time to run a callback at that time.
         *
         * @param time     The time to run the callback (seconds)
         * @param callback The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addTemporalMarker(TimeProducer time, MarkerCallback callback) {
            mirroredBuilder.addTemporalMarker(time, callback);
            return super.addTemporalMarker(time, callback);
        }

        /**
         * Add a temporal marker at a given time to run a callback at that time.
         *
         * @param time     The time to run the callback
         * @param timeUnit The unit of the time supplied (will be converted to seconds)
         * @param callback The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addTemporalMarker(TimeProducer time, Time timeUnit, MarkerCallback callback) {
            mirroredBuilder.addTemporalMarker(time, timeUnit, callback);
            return super.addTemporalMarker(time, timeUnit, callback);
        }

        /**
         * Add a spatial marker at the current position to run a callback at that position.
         *
         * @param pointInches The point to run the callback (inches)
         * @param callback    The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addSpatialMarker(Vector2d pointInches, MarkerCallback callback) {
            mirroredBuilder.addSpatialMarker(new Vector2d(pointInches.getX(), -pointInches.getY()), callback);
            return super.addSpatialMarker(pointInches, callback);
        }

        /**
         * Add a spatial marker at the current position to run a callback at that position.
         *
         * @param point    The point to run the callback
         * @param inUnit   The unit of the point (will be converted to inches)
         * @param callback The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addSpatialMarker(Vector2d point, Distance inUnit, MarkerCallback callback) {
            mirroredBuilder.addSpatialMarker(new Vector2d(point.getX(), -point.getY()), callback);
            return super.addSpatialMarker(point, inUnit, callback);
        }

        /**
         * Add a displacement marker at the current displacement to run a callback at that displacement.
         *
         * @param callback The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addDisplacementMarker(MarkerCallback callback) {
            mirroredBuilder.addDisplacementMarker(callback);
            return super.addDisplacementMarker(callback);
        }

        /**
         * Add a displacement marker at the current displacement plus an offset to run a callback at that displacement.
         *
         * @param offsetInches The offset to add to the current displacement (inches)
         * @param callback     The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder UNSTABLE_addDisplacementMarkerOffset(double offsetInches, MarkerCallback callback) {
            mirroredBuilder.UNSTABLE_addDisplacementMarkerOffset(offsetInches, callback);
            return super.UNSTABLE_addDisplacementMarkerOffset(offsetInches, callback);
        }

        /**
         * Add a displacement marker at the current displacement plus an offset to run a callback at that displacement.
         *
         * @param offset   The offset to add to the current displacement
         * @param inUnit   The unit of the offset (will be converted to inches)
         * @param callback The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder UNSTABLE_addDisplacementMarkerOffset(double offset, Distance inUnit, MarkerCallback callback) {
            mirroredBuilder.UNSTABLE_addDisplacementMarkerOffset(offset, inUnit, callback);
            return super.UNSTABLE_addDisplacementMarkerOffset(offset, inUnit, callback);
        }

        /**
         * Add a displacement marker at a given displacement to run a callback at that displacement.
         *
         * @param displacementInches The displacement to run the callback (inches)
         * @param callback           The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addDisplacementMarker(double displacementInches, MarkerCallback callback) {
            mirroredBuilder.addDisplacementMarker(displacementInches, callback);
            return super.addDisplacementMarker(displacementInches, callback);
        }

        /**
         * Add a displacement marker at a given displacement to run a callback at that displacement.
         *
         * @param displacement The displacement to run the callback
         * @param inUnit       The unit of the displacement (will be converted to inches)
         * @param callback     The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addDisplacementMarker(double displacement, Distance inUnit, MarkerCallback callback) {
            mirroredBuilder.addDisplacementMarker(displacement, inUnit, callback);
            return super.addDisplacementMarker(displacement, inUnit, callback);
        }

        /**
         * Add a displacement marker at a given displacement to run a callback at that displacement.
         *
         * @param scale        A multiplicative scale to apply to the displacement
         * @param offsetInches The offset to add to the displacement (inches)
         * @param callback     The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addDisplacementMarker(double scale, double offsetInches, MarkerCallback callback) {
            mirroredBuilder.addDisplacementMarker(scale, offsetInches, callback);
            return super.addDisplacementMarker(scale, offsetInches, callback);
        }

        /**
         * Add a displacement marker at a given displacement to run a callback at that displacement.
         *
         * @param scale    A multiplicative scale to apply to the displacement
         * @param offset   The offset to add to the displacement
         * @param inUnit   The unit of the offset (will be converted to inches)
         * @param callback The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addDisplacementMarker(double scale, double offset, Distance inUnit, MarkerCallback callback) {
            mirroredBuilder.addDisplacementMarker(scale, offset, inUnit, callback);
            return super.addDisplacementMarker(scale, offset, inUnit, callback);
        }

        /**
         * Add a displacement marker at a given displacement to run a callback at that displacement.
         *
         * @param displacementInches The displacement to run the callback (inches)
         * @param callback           The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addDisplacementMarker(DisplacementProducer displacementInches, MarkerCallback callback) {
            mirroredBuilder.addDisplacementMarker(displacementInches, callback);
            return super.addDisplacementMarker(displacementInches, callback);
        }

        /**
         * Add a displacement marker at a given displacement to run a callback at that displacement.
         *
         * @param displacement The displacement to run the callback
         * @param inUnit       The unit of the displacement (will be converted to inches)
         * @param callback     The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addDisplacementMarker(DisplacementProducer displacement, Distance inUnit, MarkerCallback callback) {
            mirroredBuilder.addDisplacementMarker(displacement, inUnit, callback);
            return super.addDisplacementMarker(displacement, inUnit, callback);
        }

        /**
         * Turn to a given angle.
         *
         * @param radians The angle to turn (radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder turn(double radians) {
            mirroredBuilder.turn(Mathf.normaliseAngle(Radians.of(radians).plus(Radians.of(Math.PI))).in(Radians));
            return super.turn(radians);
        }

        /**
         * Turn to a given angle.
         *
         * @param angle     The angle to turn
         * @param angleUnit The unit of the angle (will be converted to radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder turn(double angle, Angle angleUnit) {
            mirroredBuilder.turn(Mathf.normaliseAngle(angleUnit.of(angle).plus(Radians.of(Math.PI))).in(angleUnit), angleUnit);
            return super.turn(angle, angleUnit);
        }

        /**
         * Turn to a given angle with custom maximum angular velocity and acceleration.
         *
         * @param angle       The angle to turn (radians)
         * @param maxAngVel   The maximum angular velocity (radians/sec)
         * @param maxAngAccel The maximum angular acceleration (radians/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder turn(double angle, double maxAngVel, double maxAngAccel) {
            mirroredBuilder.turn(Mathf.normaliseAngle(Radians.of(angle).plus(Radians.of(Math.PI))).in(Radians), maxAngVel, maxAngAccel);
            return super.turn(angle, maxAngVel, maxAngAccel);
        }

        /**
         * Turn to a given angle with custom maximum angular velocity and acceleration.
         *
         * @param angle       The angle to turn
         * @param angleUnit   The unit of the angle (will be converted to radians)
         * @param maxAngVel   The maximum angular velocity
         * @param velUnit     The unit of the maximum angular velocity (will be converted to radians/sec)
         * @param maxAngAccel The maximum angular acceleration
         * @param accelUnit   The unit of the maximum angular acceleration (will be converted to radians/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder turn(double angle, Angle angleUnit, double maxAngVel, Velocity<Angle> velUnit, double maxAngAccel, Velocity<Velocity<Angle>> accelUnit) {
            mirroredBuilder.turn(Mathf.normaliseAngle(angleUnit.of(angle).plus(Radians.of(Math.PI))).in(Radians), angleUnit, maxAngVel, velUnit, maxAngAccel, accelUnit);
            return super.turn(angle, angleUnit, maxAngVel, velUnit, maxAngAccel, accelUnit);
        }

        /**
         * Wait for a given number of seconds.
         *
         * @param seconds The number of seconds to wait
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder waitSeconds(double seconds) {
            mirroredBuilder.waitSeconds(seconds);
            return super.waitSeconds(seconds);
        }

        /**
         * Wait for a given amount of time.
         *
         * @param time The amount of time to wait.
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder waitFor(Measure<Time> time) {
            mirroredBuilder.waitFor(time);
            return super.waitFor(time);
        }

        /**
         * Add a trajectory to the sequence.
         *
         * @param trajectory The trajectory to add
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addTrajectory(Trajectory trajectory) {
            mirroredBuilder.addTrajectory(trajectory);
            return super.addTrajectory(trajectory);
        }

        /**
         * Set a timeout for the trajectory, to be applied to the overhead task running the trajectory.
         * Should be called first, before any other builder methods.
         * If this method is not called or fed negative values, an infinite timeout will be used.
         *
         * @param interval Timeout for the trajectory
         * @return trajectory builder
         */
        public RoadRunnerTrajectoryTaskBuilder withTimeout(Measure<Time> interval) {
            if (interval.lt(INFINITE_TIMEOUT)) {
                return this;
            }
            timeout = interval;
            return this;
        }

        /**
         * Set the task name of the trajectory to show up in the telemetry.
         *
         * @param taskName Name of the task
         * @return trajectory builder
         */
        public RoadRunnerTrajectoryTaskBuilder withName(String taskName) {
            name = taskName;
            return this;
        }

        /**
         * Set the priority level of the task.
         *
         * @param p Priority level
         * @return trajectory builder
         */
        public RoadRunnerTrajectoryTaskBuilder withPriority(PriorityLevel p) {
            priority = p;
            return this;
        }

        /**
         * Mirror a {@link TrajectorySequence} into the given {@link Reference}, which will effectively "flip" over the y-axis from the center of the field.
         * This method is useful for trajectories that need to be run in the opposite direction, such as a Red Left trajectory
         * being flipped to a Blue Right trajectory.
         *
         * @param out Reference to mirror the trajectory to - this will be mirrored upon a build call
         * @return Builder,
         */
        public RoadRunnerTrajectoryTaskBuilder mirrorToRef(Reference<TrajectorySequence> out) {
            mirroredTrajectory = out;
            return this;
        }

        /**
         * Run a sequence of trajectories, useful if you do not need to build a trajectory but
         * run one directly. This method will override any previous trajectories added to the builder,
         * but will still mirror the sequence and accept task settings.
         * <p>
         * Example usage:
         * {@code
         *     makeTrajectory().runSequence(sequence).withName("pre-built sequence").addTask();
         * }
         *
         * @param sequence The sequence to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder runSequence(TrajectorySequence sequence) {
            overrideSequence = sequence;
            return this;
        }

        /**
         * Build the trajectory sequence (and mirrored sequence) without creating a task.
         * This method is useful if you are not running an {@link AutonomousBunyipsOpMode}.
         *
         * @return The built trajectory sequence from the builder, with no task being created or added.
         */
        public TrajectorySequence build() {
            if (overrideSequence != null)
                return overrideSequence;
            if (mirroredTrajectory != null)
                mirroredTrajectory.set(mirroredBuilder.build());
            return super.build();
        }

        /**
         * Build the trajectory sequence and task for use by the user.
         * This method is useful is you are running this task as part of a {@link TaskGroup}, or if you are
         * not running an {@link AutonomousBunyipsOpMode} and cannot use the queue system.
         * <p>
         * This task will be added to the global poses list, so the next implicitly created trajectory will
         * start from the end of this one.
         *
         * @return The built task, not added to the task queue automatically.
         * @see #buildTask(boolean)
         */
        public RoadRunnerTask<RoadRunnerDrive> buildTask() {
            return buildTask(true);
        }

        /**
         * Build the trajectory sequence and task for use by the user.
         * This method is useful is you are running this task as part of a {@link TaskGroup}, or if you are
         * not running an {@link AutonomousBunyipsOpMode} and cannot use the queue system.
         *
         * @param addToGlobalPoses Whether to add the trajectory to the rrTasks list, therefore making the next implicit
         *                         trajectory start from the end of this one
         * @return The built task, not added to the task queue automatically.
         */
        public RoadRunnerTask<RoadRunnerDrive> buildTask(boolean addToGlobalPoses) {
            TrajectorySequence sequence = overrideSequence != null ? overrideSequence : build();
            RoadRunnerTask<RoadRunnerDrive> task = new RoadRunnerTask<>(timeout, drive, sequence);
            task.withTimeout(timeout);
            task.withName(name);
            if (addToGlobalPoses)
                rrTasks.add(task);
            return task;
        }

        /**
         * Build the trajectory sequence and task, and add it automatically to the {@link AutonomousBunyipsOpMode}
         * task queue. This method is useful if you are running this task as a standalone task, and will only
         * work if you are running an {@link AutonomousBunyipsOpMode}.
         */
        public void addTask() {
            RoadRunnerTask<RoadRunnerDrive> task = buildTask();
            switch (priority) {
                case LAST:
                    ((AutonomousBunyipsOpMode) BunyipsOpMode.getInstance()).addTaskLast(task);
                    break;
                case NORMAL:
                    ((AutonomousBunyipsOpMode) BunyipsOpMode.getInstance()).addTask(task);
                    break;
                case FIRST:
                    ((AutonomousBunyipsOpMode) BunyipsOpMode.getInstance()).addTaskFirst(task);
                    break;
            }
        }
    }
}
