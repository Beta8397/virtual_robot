package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;
import static org.murraybridgebunyips.bunyipslib.tasks.bases.Task.INFINITE_TIMEOUT;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.DisplacementProducer;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.TimeProducer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.external.units.Velocity;
import org.murraybridgebunyips.bunyipslib.roadrunner.Limit;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.murraybridgebunyips.bunyipslib.tasks.RoadRunnerTask;
import org.murraybridgebunyips.bunyipslib.tasks.groups.TaskGroup;

import java.util.Objects;

/**
 * RoadRunner utility interface for autonomous OpModes. Implement this interface in a {@link AutonomousBunyipsOpMode}.
 * Do not override any of the default methods in this interface, as they are used for RoadRunner task scheduling.
 * <p>
 * Previously named RoadRunnerAutonomousBunyipsOpMode (RRABOM, nickname "Rabone").
 * <p>
 * <i>This interface may also be used in a normal {@link BunyipsOpMode}, however the {@code addTask()} builder method
 * will not work as it requires the presence of {@link AutonomousBunyipsOpMode}.</i>
 *
 * @author Lucas Bubner, 2024
 * @noinspection InterfaceMayBeAnnotatedFunctional
 * @see AutonomousBunyipsOpMode
 */
public interface RoadRunner {
    /**
     * Whether timeouts for built tasks should be force-set to no timeout. By default tasks are set to the trajectory
     * duration but some systems do not want this restriction, therefore this exists to set that behaviour.
     * You can call this directly to set it. It is auto-reset at the start of every BunyipsOpMode.
     */
    Reference<Boolean> noTimeouts = Reference.of(false);
    /**
     * Stores last spliced pose.
     */
    Reference<Pose2d> splicedPose = Reference.empty();

    /**
     * Reset fields for an OpMode.
     */
    static void resetForOpMode() {
        splicedPose.clear();
        noTimeouts.set(false);
    }

    /**
     * Convert a Pose2d of specified units to inches and radians.
     * Convenient for having unit-defined poses that need to be of the same unit of default RoadRunner poses, such
     * as adding vectors for relative movement.
     *
     * @param pose         The pose to convert
     * @param distanceUnit The unit of the pose's distance
     * @param angleUnit    The unit of the pose's angle
     * @return The converted pose
     */
    static Pose2d unitPose(Pose2d pose, Distance distanceUnit, Angle angleUnit) {
        return unitPose(pose, distanceUnit, angleUnit, 1);
    }

    /**
     * Convert a Vector2d of specified units to inches and radians.
     * Convenient for having unit-defined poses that need to be of the same unit of default RoadRunner poses, such
     * as adding vectors for relative movement.
     *
     * @param vector The vector to convert
     * @param unit   The unit of the vector
     * @return The converted vector
     */
    static Vector2d unitVec(Vector2d vector, Distance unit) {
        return unitVec(vector, unit, 1);
    }

    /**
     * Convert a Pose2d of specified units to inches and radians.
     * Convenient for having unit-defined poses that need to be of the same unit of default RoadRunner poses, such
     * as adding vectors for relative movement.
     *
     * @param pose           The pose to convert
     * @param distanceUnit   The unit of the pose's distance
     * @param angleUnit      The unit of the pose's angle
     * @param scalarDistMult The vector scalar, in the unit of distanceUnit.
     * @return The converted pose
     */
    static Pose2d unitPose(Pose2d pose, Distance distanceUnit, Angle angleUnit, double scalarDistMult) {
        return new Pose2d(
                Inches.convertFrom(pose.getX() * scalarDistMult, distanceUnit),
                Inches.convertFrom(pose.getY() * scalarDistMult, distanceUnit),
                Radians.convertFrom(pose.getHeading(), angleUnit)
        );
    }

    /**
     * Convert a Vector2d of specified units to inches and radians.
     * Convenient for having unit-defined poses that need to be of the same unit of default RoadRunner poses, such
     * as adding vectors for relative movement.
     *
     * @param vector         The vector to convert
     * @param unit           The unit of the vector
     * @param scalarDistMult The vector scalar, in the unit of the unit parameter.
     * @return The converted vector
     */
    static Vector2d unitVec(Vector2d vector, Distance unit, double scalarDistMult) {
        return new Vector2d(
                Inches.convertFrom(vector.getX() * scalarDistMult, unit),
                Inches.convertFrom(vector.getY() * scalarDistMult, unit)
        );
    }

    /**
     * Mirror a global pose across the alliance plane. This negates the heading and y components.
     *
     * @param pose the pose to mirror
     * @return the mirrored pose
     */
    static Pose2d mirror(Pose2d pose) {
        return new Pose2d(pose.getX(), -pose.getY(), -pose.getHeading());
    }

    /**
     * Mirror a global vector across the alliance plane. This negates the y component.
     *
     * @param vector the vector to mirror
     * @return the mirrored vector
     */
    static Vector2d mirror(Vector2d vector) {
        return new Vector2d(vector.getX(), -vector.getY());
    }

    /**
     * Get the drive instance reference to be used for RoadRunner trajectories.
     * <b>Do NOT instantiate a new drive instance here, use the reference from your subsystems in your OpMode.</b>
     * (e.g. {@code return drive;}) instead of ({@code return new MecanumDrive(...);})
     *
     * @return Drive instance reference
     */
    @NonNull
    RoadRunnerDrive getDrive();

    /**
     * Reset pose info back to default.
     */
    default void resetPoseInfo() {
        resetForOpMode();
        Storage.memory().lastKnownPosition = null;
        getDrive().setPoseEstimate(new Pose2d());
    }

    /**
     * Make a translation velocity constraint.
     *
     * @param translation The translation velocity
     * @param unit        The unit of the translation velocity
     * @return The velocity constraint
     */
    default TrajectoryVelocityConstraint atVelocity(double translation, Velocity<Distance> unit) {
        return Limit.ofVelocity(unit.of(translation), getDrive());
    }

    /**
     * Make an angular velocity constraint.
     *
     * @param rotation The angular velocity
     * @param unit     The unit of the angular velocity
     * @return The velocity constraint
     */
    default TrajectoryVelocityConstraint atAngularVelocity(double rotation, Velocity<Angle> unit) {
        return Limit.ofAngularVelocity(unit.of(rotation), getDrive());
    }

    /**
     * Make a translation and angular velocity constraint.
     *
     * @param translation     The translation velocity
     * @param translationUnit The unit of the translation velocity
     * @param rotation        The angular velocity
     * @param rotationUnit    The unit of the angular velocity
     * @return The velocity constraint
     */
    default TrajectoryVelocityConstraint atVelocities(double translation, Velocity<Distance> translationUnit, double rotation, Velocity<Angle> rotationUnit) {
        return Limit.ofVelocities(translationUnit.of(translation), rotationUnit.of(rotation), getDrive());
    }

    /**
     * Make a translation acceleration constraint.
     *
     * @param acceleration The translation acceleration
     * @param unit         The unit of the translation acceleration
     * @return The acceleration constraint
     */
    default TrajectoryAccelerationConstraint atAcceleration(double acceleration, Velocity<Velocity<Distance>> unit) {
        return Limit.ofAcceleration(unit.of(acceleration), getDrive());
    }

    /**
     * Set the current pose estimate of the drive. This method is useful when parsing the user-selected start
     * position, so field-relative trajectories can be built.
     *
     * @param poseEstimateInchRad The pose estimate to set (in, in, rad)
     */
    default void setPose(Pose2d poseEstimateInchRad) {
        Pose2d curr = getDrive().getPoseEstimate();
        getDrive().setPoseEstimate(poseEstimateInchRad);
        Dbg.logd(getClass(), "Pose estimate updated: % -> %", curr, poseEstimateInchRad);
    }

    /**
     * Set the current pose estimate of the drive. This method is useful when parsing the user-selected start
     * position, so field-relative trajectories can be built.
     *
     * @param poseEstimate The pose estimate to set
     * @param inUnit       The unit of the end pose vector (will be converted to inches)
     * @param angleUnit    The unit of the end pose heading (will be converted to radians)
     */
    default void setPose(Pose2d poseEstimate, Distance inUnit, Angle angleUnit) {
        double x = Inches.convertFrom(poseEstimate.getX(), inUnit);
        double y = Inches.convertFrom(poseEstimate.getY(), inUnit);
        double r = Radians.convertFrom(poseEstimate.getHeading(), angleUnit);
        setPose(new Pose2d(x, y, r));
    }

    // Unfortunately, we need so many overrides for makeTrajectory as there are different unit constructions plus an additional parameter
    // to whether this pose should be mirrored if used in a mirrorToRef. We can't ask for this information later as we need it now, so
    // this is our best solution. Fortunately, the builder can defer this in a nice sequence of .disableMirroring().enableMirroring(),
    // but at least this extra parameter gives us the option to control the mirroring of every different path segment.

    /**
     * Use this method to build a new RoadRunner trajectory or to add a RoadRunner trajectory to the task queue.
     *
     * @param startPoseInchRad Starting pose of the trajectory to be built starting at (in, in, rad). This pose will consequently used as the next implicit pose.
     * @return Builder for the trajectory
     */
    default RoadRunnerTrajectoryTaskBuilder makeTrajectory(Pose2d startPoseInchRad) {
        return makeTrajectory(startPoseInchRad, false);
    }

    /**
     * Use this method to build a new RoadRunner trajectory or to add a RoadRunner trajectory to the task queue.
     *
     * @param startPoseInchRad Starting pose of the trajectory to be built starting at (in, in, rad). This pose will consequently used as the next implicit pose.
     * @param mirrorPoseRef    Whether to mirror this starting pose if used in a {@code mirrorToRef}. This is disabled by default to account for global coordinate systems.
     * @return Builder for the trajectory
     */
    default RoadRunnerTrajectoryTaskBuilder makeTrajectory(Pose2d startPoseInchRad, boolean mirrorPoseRef) {
        // noinspection rawtypes
        TrajectorySequenceBuilder builder = getDrive().trajectorySequenceBuilder(startPoseInchRad);
        return new RoadRunnerTrajectoryTaskBuilder(getDrive(), startPoseInchRad, builder.getBaseVelConstraint(), builder.getBaseAccelConstraint(), builder.getBaseTurnConstraintMaxAngVel(), builder.getBaseTurnConstraintMaxAngAccel(), mirrorPoseRef);
    }

    /**
     * Use this method to build a new RoadRunner trajectory or to add a RoadRunner trajectory to the task queue.
     *
     * @param startPose     Starting pose of the trajectory to be built starting at. This pose will consequently used as the next implicit pose.
     * @param inUnit        The unit of the end pose vector (will be converted to inches)
     * @param angleUnit     The unit of the end pose heading (will be converted to radians)
     * @param mirrorPoseRef Whether to mirror this starting pose if used in a {@code mirrorToRef}. This is disabled by default to account for global coordinate systems.
     * @return Builder for the trajectory
     */
    default RoadRunnerTrajectoryTaskBuilder makeTrajectory(Pose2d startPose, Distance inUnit, Angle angleUnit, boolean mirrorPoseRef) {
        double x = Inches.convertFrom(startPose.getX(), inUnit);
        double y = Inches.convertFrom(startPose.getY(), inUnit);
        double r = Radians.convertFrom(startPose.getHeading(), angleUnit);
        return makeTrajectory(new Pose2d(x, y, r), mirrorPoseRef);
    }

    /**
     * Use this method to build a new RoadRunner trajectory or to add a RoadRunner trajectory to the task queue.
     *
     * @param startPose Starting pose of the trajectory to be built starting at. This pose will consequently used as the next implicit pose.
     * @param inUnit    The unit of the end pose vector (will be converted to inches)
     * @param angleUnit The unit of the end pose heading (will be converted to radians)
     * @return Builder for the trajectory
     */
    default RoadRunnerTrajectoryTaskBuilder makeTrajectory(Pose2d startPose, Distance inUnit, Angle angleUnit) {
        return makeTrajectory(startPose, inUnit, angleUnit, false);
    }

    /**
     * Use this method to build a new RoadRunner trajectory or to add a RoadRunner trajectory to the task queue.
     * Without arguments, the start pose will use the current pose estimate of the drive *or* the last spliced pose as the starting
     * pose of the trajectory. If there is no buffered spliced pose, the current pose estimate will be used.
     *
     * @param mirrorPoseRef Whether to mirror the implicit starting pose if used in a {@code mirrorToRef}. This is disabled by default to account for global coordinate systems.
     * @return Builder for the trajectory
     * @see #makeTrajectory(Pose2d)
     */
    default RoadRunnerTrajectoryTaskBuilder makeTrajectory(boolean mirrorPoseRef) {
        // If we're using an implicit start pose in the presence of a lastKnownPosition, it is likely the case that
        // we don't want to use the lastKnownPosition as the implicit pose, so we'll reset the pose info here
        Pose2d dp = getDrive().getPoseEstimate();
        Pose2d lastKnown = Storage.memory().lastKnownPosition;
        if (lastKnown != null && splicedPose.isNull() && dp.epsilonEquals(lastKnown))
            resetPoseInfo();
        dp = getDrive().getPoseEstimate();
        Pose2d implicitPose = splicedPose.isNotNull() ? splicedPose.get() : dp;
        // noinspection rawtypes
        TrajectorySequenceBuilder builder = getDrive().trajectorySequenceBuilder(implicitPose);
        return new RoadRunnerTrajectoryTaskBuilder(getDrive(), implicitPose, builder.getBaseVelConstraint(), builder.getBaseAccelConstraint(), builder.getBaseTurnConstraintMaxAngVel(), builder.getBaseTurnConstraintMaxAngAccel(), mirrorPoseRef);
    }

    /**
     * Use this method to build a new RoadRunner trajectory or to add a RoadRunner trajectory to the task queue.
     * Without arguments, the start pose will use the current pose estimate of the drive *or* the last spliced pose as the starting
     * pose of the trajectory. If there is no buffered spliced pose, the current pose estimate will be used.
     *
     * @return Builder for the trajectory
     * @see #makeTrajectory(Pose2d)
     */
    default RoadRunnerTrajectoryTaskBuilder makeTrajectory() {
        return makeTrajectory(false);
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
        private boolean mirroring = true;
        private double mult = 1;
        private TrajectorySequence overrideSequence;
        private Measure<Time> timeout = null;
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
         * @param mirrorPoseRef                 Whether to mirror the startPose if used in a {@link #mirrorToRef}.
         */
        public RoadRunnerTrajectoryTaskBuilder(RoadRunnerDrive drive, Pose2d startPose, Double startTangent, TrajectoryVelocityConstraint baseVelConstraint, TrajectoryAccelerationConstraint baseAccelConstraint, double baseTurnConstraintMaxAngVel, double baseTurnConstraintMaxAngAccel, boolean mirrorPoseRef) {
            super(startPose, startTangent, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
            Pose2d pose = splicedPose.isNotNull() ? Objects.requireNonNull(splicedPose.get()) : startPose;
            mirroredBuilder = new TrajectorySequenceBuilder<>(mirrorPoseRef ? internalMirror(pose) : pose, -startTangent, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
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
         * @param mirrorPoseRef                 Whether to mirror the startPose if used in a {@link #mirrorToRef}.
         */
        public RoadRunnerTrajectoryTaskBuilder(RoadRunnerDrive drive, Pose2d startPose, TrajectoryVelocityConstraint baseVelConstraint, TrajectoryAccelerationConstraint baseAccelConstraint, double baseTurnConstraintMaxAngVel, double baseTurnConstraintMaxAngAccel, boolean mirrorPoseRef) {
            super(startPose, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
            Pose2d pose = splicedPose.isNotNull() ? Objects.requireNonNull(splicedPose.get()) : startPose;
            mirroredBuilder = new TrajectorySequenceBuilder<>(mirrorPoseRef ? internalMirror(pose) : pose, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
            this.drive = drive;
        }

        private Pose2d internalMirror(Pose2d pose) {
            if (!mirroring) return pose;
            return mirror(pose);
        }

        private Vector2d internalMirror(Vector2d vector) {
            if (!mirroring) return vector;
            return mirror(vector);
        }

        /**
         * Toggle {@link #mirrorToRef} mirroring for the following builder instructions.
         * <p>
         * This is useful when you want to temporarily rely on a global field coordinate, instead of having it flip
         * in the mirrored trajectory. This disabling is done automatically in the constructor and can be enabled with a parameter, whereas actual segments
         * are enabled to be mirrored by default and can be temporarily toggled with this method.
         *
         * @param shouldMirror whether the mirror reference should mirror the following builder instructions, by default this is on
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder setRefMirroring(boolean shouldMirror) {
            mirroring = shouldMirror;
            return this;
        }

        /**
         * Set the scale to multiply route <i>distances</i> by for the following builder instructions.
         * Note that this scale will unconditionally apply to all instructions following this call, until this
         * builder ends or this method is called again. Ensure your distance units are consistent before multiplying.
         *
         * @param scale the multiplicative scale to use for further builder instructions,
         *              which will scale distance but not heading instructions as they are accurate
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder setScale(double scale) {
            mult = scale;
            return this;
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
            endPositionInches = endPositionInches.times(mult);
            mirroredBuilder.lineTo(internalMirror(endPositionInches), velConstraint, accelConstraint);
            return super.lineTo(endPositionInches, velConstraint, accelConstraint);
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
            endPositionInches = endPositionInches.times(mult);
            mirroredBuilder.lineToConstantHeading(internalMirror(endPositionInches), velConstraint, accelConstraint);
            return super.lineToConstantHeading(endPositionInches, velConstraint, accelConstraint);
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
            endPoseInchRad = new Pose2d(endPoseInchRad.vec().times(mult), endPoseInchRad.getHeading());
            mirroredBuilder.lineToLinearHeading(internalMirror(endPoseInchRad), velConstraint, accelConstraint);
            return super.lineToLinearHeading(endPoseInchRad, velConstraint, accelConstraint);
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
            endPoseInchRad = new Pose2d(endPoseInchRad.vec().times(mult), endPoseInchRad.getHeading());
            mirroredBuilder.lineToSplineHeading(internalMirror(endPoseInchRad), velConstraint, accelConstraint);
            return super.lineToSplineHeading(endPoseInchRad, velConstraint, accelConstraint);
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
            endPositionInches = endPositionInches.times(mult);
            mirroredBuilder.strafeTo(internalMirror(endPositionInches), velConstraint, accelConstraint);
            return super.strafeTo(endPositionInches, velConstraint, accelConstraint);
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
            inches *= mult;
            mirroredBuilder.forward(inches, velConstraint, accelConstraint);
            return super.forward(inches, velConstraint, accelConstraint);
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
            inches *= mult;
            mirroredBuilder.back(inches, velConstraint, accelConstraint);
            return super.back(inches, velConstraint, accelConstraint);
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
            inches *= mult;
            mirroredBuilder.strafeLeft(inches * (mirroring ? -1 : 1), velConstraint, accelConstraint);
            return super.strafeLeft(inches, velConstraint, accelConstraint);
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
            inches *= mult;
            mirroredBuilder.strafeRight(inches * (mirroring ? -1 : 1), velConstraint, accelConstraint);
            return super.strafeRight(inches, velConstraint, accelConstraint);
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
            endPositionInches = endPositionInches.times(mult);
            mirroredBuilder.splineTo(internalMirror(endPositionInches), endHeadingRad * (mirroring ? -1 : 1), velConstraint, accelConstraint);
            return super.splineTo(endPositionInches, endHeadingRad, velConstraint, accelConstraint);
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
            endPositionInches = endPositionInches.times(mult);
            mirroredBuilder.splineToConstantHeading(internalMirror(endPositionInches), endHeadingRad * (mirroring ? -1 : 1), velConstraint, accelConstraint);
            return super.splineToConstantHeading(endPositionInches, endHeadingRad, velConstraint, accelConstraint);
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
            endPoseInchRad = endPoseInchRad.times(mult);
            mirroredBuilder.splineToLinearHeading(internalMirror(endPoseInchRad), endHeadingRad * (mirroring ? -1 : 1), velConstraint, accelConstraint);
            return super.splineToLinearHeading(endPoseInchRad, endHeadingRad, velConstraint, accelConstraint);
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
            endPoseInchRad = new Pose2d(endPoseInchRad.vec().times(mult), endPoseInchRad.getHeading());
            mirroredBuilder.splineToSplineHeading(internalMirror(endPoseInchRad), endPoseInchRad.getHeading() * (mirroring ? -1 : 1), velConstraint, accelConstraint);
            return super.splineToSplineHeading(endPoseInchRad, endHeadingRad, velConstraint, accelConstraint);
        }

        /**
         * Set the tangent of the next path.
         *
         * @param tangent The tangent (radians)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder setTangent(double tangent) {
            mirroredBuilder.setTangent(tangent * (mirroring ? -1 : 1));
            return super.setTangent(tangent);
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
         * Reset the turn constraints to the base turn constraints.
         *
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder resetTurnConstraint() {
            mirroredBuilder.resetTurnConstraint();
            return super.resetTurnConstraint();
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
         * Add a spatial marker at the current position to run a callback at that position.
         *
         * @param pointInches The point to run the callback (inches)
         * @param callback    The callback to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addSpatialMarker(Vector2d pointInches, MarkerCallback callback) {
            mirroredBuilder.addSpatialMarker(internalMirror(pointInches), callback);
            return super.addSpatialMarker(pointInches, callback);
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
         * Turn to a given angle with custom maximum angular velocity and acceleration.
         *
         * @param radians     The angle to turn (radians)
         * @param maxAngVel   The maximum angular velocity (radians/sec)
         * @param maxAngAccel The maximum angular acceleration (radians/sec^2)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder turn(double radians, double maxAngVel, double maxAngAccel) {
            mirroredBuilder.turn(radians * (mirroring ? -1 : 1), maxAngVel, maxAngAccel);
            return super.turn(radians, maxAngVel, maxAngAccel);
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
         * Add a trajectory to the sequence.
         * Note that {@link #mirrorToRef} will not mirror this trajectory as it is already built and cannot be modified.
         *
         * @param trajectory The trajectory to add
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder addTrajectory(Trajectory trajectory) {
            // Trajectories that are built cannot be mirrored, so we have to use the non-mirrored builder
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
         * <p>
         * NOTE: The Starting Pose (builder parameter) is <b>NOT</b> flipped with using mirrorToRef() by default.
         * This is to allow global coordinate systems to work properly. If you wish to mirror the Starting Pose,
         * you must pass an additional argument of {@code true} to {@link #makeTrajectory()}, which will flip the starting
         * pose.
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
         * run one directly. This method will override and void any previous trajectory added to this builder,
         * but will still accept task settings and construction.
         * <p>
         * By default, with no extra boolean parameter, this method will also set the Pose Estimate of the drive to
         * the start pose of the sequence immediately. Pass in false to disable this behaviour.
         * <p>
         * Example usage:
         * {@code
         * makeTrajectory([optional pose estimate to set to on task build]).runSequence(sequence).withName("pre-built sequence").addTask();
         * }
         *
         * @param sequence The sequence to run
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder runSequence(TrajectorySequence sequence) {
            return runSequence(sequence, true);
        }

        /**
         * Run a sequence of trajectories, useful if you do not need to build a trajectory but
         * run one directly. This method will override and void any previous trajectory added to this builder,
         * but will still accept task settings and construction.
         * <p>
         * This method can optionally also set the Pose Estimate of the drive to
         * the start pose of the sequence immediately. Pass in false to disable this behavior.
         * <p>
         * Example usage:
         * {@code
         * makeTrajectory([optional pose estimate to set to on task build]).runSequence(sequence).withName("pre-built sequence").addTask();
         * }
         *
         * @param sequence            The sequence to run
         * @param setDrivePoseToStart Whether to set the drive pose estimate to the start of the sequence (immediately)
         * @return The builder
         */
        public RoadRunnerTrajectoryTaskBuilder runSequence(TrajectorySequence sequence, boolean setDrivePoseToStart) {
            overrideSequence = sequence;
            if (setDrivePoseToStart)
                drive.setPoseEstimate(sequence.start());
            return this;
        }

        /**
         * Build the trajectory sequence (and mirrored sequence) without creating a task.
         * This method is useful if you are not running an {@link AutonomousBunyipsOpMode}.
         * <p>
         * Note that building the trajectory itself not update any implicit poses for further calls.
         * See {@link #buildTask()}/{@link #addTask()} for this behaviour.
         *
         * @return The built trajectory sequence from the builder, with no task being created or added.
         * If there is an {@link #runSequence(TrajectorySequence) override sequence}, that will be returned instead.
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
         * start from the end of this one. Set {@code useEndAsNextImplicitPose} to false if you do not want this behavior,
         * such as if there are side effects of construction using multiple implicit trajectories.
         *
         * @return The built task, not added to the task queue automatically.
         * @see #buildTask(boolean)
         */
        public RoadRunnerTask buildTask() {
            return buildTask(true);
        }

        /**
         * Build the trajectory sequence and task for use by the user.
         * This method is useful is you are running this task as part of a {@link TaskGroup}, or if you are
         * not running an {@link AutonomousBunyipsOpMode} and cannot use the queue system.
         *
         * @param useEndAsNextImplicitPose Whether to use this trajectory end point as a global splice, therefore making the
         *                                 next implicit trajectory start from the end of this one. This should be used if you are using
         *                                 makeTrajectory() and building a task, and there are no side effects of construction using
         *                                 multiple implicit trajectories.
         * @return The built task, not added to the task queue automatically.
         */
        public RoadRunnerTask buildTask(boolean useEndAsNextImplicitPose) {
            TrajectorySequence sequence = build();
            // Timeout is auto-set to the trajectory duration
            RoadRunnerTask task = new RoadRunnerTask(INFINITE_TIMEOUT, drive, sequence);
            if (Boolean.TRUE.equals(noTimeouts.get())) {
                // Global OpMode setting takes priority
                task.withTimeout(INFINITE_TIMEOUT);
            } else if (timeout != null) {
                // Override with the user's timeout if they want it
                task.withTimeout(timeout);
            }
            task.withName(name);
            if (useEndAsNextImplicitPose)
                splicedPose.set(sequence.end());
            return task;
        }

        /**
         * Build the trajectory sequence and task, and add it automatically to the {@link AutonomousBunyipsOpMode}
         * task queue. This method is useful if you are running this task as a standalone task, and will only
         * work if you are running an {@link AutonomousBunyipsOpMode}. This trajectory will be added to the global
         * poses list, so the next implicitly created trajectory will start from the end of this one.
         */
        public void addTask() {
            RoadRunnerTask task = buildTask();
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