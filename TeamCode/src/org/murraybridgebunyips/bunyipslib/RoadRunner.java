package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;
import static org.murraybridgebunyips.bunyipslib.tasks.bases.Task.INFINITE_TIMEOUT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.murraybridgebunyips.bunyipslib.tasks.RoadRunnerTask;

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
     * Use this method to build a new RoadRunner trajectory to the queue.
     * Creates a new builder for a RoadRunner trajectory, which will automatically add a
     * task to the queue when build() is called, optionally with a timeout control ({@link RoadRunnerTrajectoryTaskBuilder#withTimeout(Measure)}).
     * <p>
     * This method is the combination of {@link #newTrajectorySequence()} and {@link #addTrajectory(TrajectorySequence)},
     * using a custom builder that supports {@code setTimeout()} and priority building.
     *
     * @param startPoseInchRad Starting pose of the trajectory, ** WILL SET DRIVE POSE ESTIMATE TO THIS POSE **, (in, in, radians)
     * @return Builder for the trajectory
     */
    default RoadRunnerTrajectoryTaskBuilder addNewTrajectory(Pose2d startPoseInchRad) {
        // noinspection rawtypes
        TrajectorySequenceBuilder builder = getDrive().trajectorySequenceBuilder(startPoseInchRad);
        getDrive().setPoseEstimate(startPoseInchRad);
        return new RoadRunnerTrajectoryTaskBuilder(getDrive(), startPoseInchRad, builder.getBaseVelConstraint(), builder.getBaseAccelConstraint(), builder.getBaseTurnConstraintMaxAngVel(), builder.getBaseTurnConstraintMaxAngAccel());
    }


    /**
     * Use this method to build a new RoadRunner trajectory to the queue.
     * Creates a new builder for a RoadRunner trajectory, which will automatically add a
     * task to the queue when build() is called, optionally with a timeout control ({@link RoadRunnerTrajectoryTaskBuilder#withTimeout(Measure)}).
     * <p>
     * This method is the combination of {@link #newTrajectorySequence()} and {@link #addTrajectory(TrajectorySequence)},
     * using a custom builder that supports {@code setTimeout()} and priority building.
     *
     * @param startPose Starting pose of the trajectory, ** WILL SET DRIVE POSE ESTIMATE TO THIS POSE **
     * @param inUnit    The unit of the end pose vector (will be converted to inches)
     * @param angleUnit The unit of the end pose heading (will be converted to radians)
     * @return Builder for the trajectory
     */
    default RoadRunnerTrajectoryTaskBuilder addNewTrajectory(Pose2d startPose, Distance inUnit, Angle angleUnit) {
        double x = Inches.convertFrom(startPose.getX(), inUnit);
        double y = Inches.convertFrom(startPose.getY(), inUnit);
        double r = Radians.convertFrom(startPose.getHeading(), angleUnit);
        return addNewTrajectory(new Pose2d(x, y, r));
    }

    /**
     * Use this method to build a new RoadRunner trajectory to the queue.
     * Creates a new builder for a RoadRunner trajectory, which will automatically add a
     * task to the queue when build() is called, optionally with a timeout control.
     * This method is the combination of {@link #newTrajectorySequence()} and {@link #addTrajectory(TrajectorySequence)},
     * using a custom builder that supports {@code setTimeout()} and priority building.
     * Without arguments, will use the current pose estimate of the drive or the last spliced pose.
     *
     * @return Builder for the trajectory
     * @see #addNewTrajectory(Pose2d)
     */
    default RoadRunnerTrajectoryTaskBuilder addNewTrajectory() {
        // noinspection rawtypes
        TrajectorySequenceBuilder builder = getDrive().trajectorySequenceBuilder(getPreviousPose());
        return new RoadRunnerTrajectoryTaskBuilder(getDrive(), getPreviousPose(), builder.getBaseVelConstraint(), builder.getBaseAccelConstraint(), builder.getBaseTurnConstraintMaxAngVel(), builder.getBaseTurnConstraintMaxAngAccel());
    }

    /**
     * Create a new builder for a RoadRunner trajectory using the drive system.
     * This will use the builder directly from RoadRunner, see {@link #addNewTrajectory(Pose2d)} for
     * integrated task building, or if you prefer you can use {@link #addTrajectory(Trajectory)}
     * to add your sequence to the queue manually.
     *
     * @return Builder for the trajectory
     * @see #newTrajectory(Pose2d)
     */
    default TrajectoryBuilder newTrajectory() {
        return getDrive().trajectoryBuilder(getPreviousPose());
    }

    /**
     * Create a new builder for a standard RoadRunner trajectory using the drive system.
     * This will use the builder directly from RoadRunner, see {@link #addNewTrajectory(Pose2d)} for
     * integrated task building, or if you prefer you can use {@link #addTrajectory(Trajectory)}
     * to add your sequence to the queue manually.
     *
     * @param startPose Starting pose of the trajectory, ** WILL SET DRIVE POSE ESTIMATE TO THIS POSE **, (in, in, radians)
     * @return Builder for the trajectory
     */
    default TrajectoryBuilder newTrajectory(Pose2d startPose) {
        getDrive().setPoseEstimate(startPose);
        return getDrive().trajectoryBuilder(startPose);
    }

    /**
     * Create a new builder for a standard RoadRunner trajectory using the drive system.
     * This will use the builder directly from RoadRunner, see {@link #addNewTrajectory(Pose2d)} for
     * integrated task building, or if you prefer you can use {@link #addTrajectory(Trajectory)}
     * to add your sequence to the queue manually.
     *
     * @param startPose Starting pose of the trajectory, ** WILL SET DRIVE POSE ESTIMATE TO THIS POSE **
     * @param inUnit    The unit of the end pose vector (will be converted to inches)
     * @param angleUnit The unit of the end pose heading (will be converted to radians)
     * @return Builder for the trajectory
     */
    default TrajectoryBuilder newTrajectory(Pose2d startPose, Distance inUnit, Angle angleUnit) {
        double x = Inches.convertFrom(startPose.getX(), inUnit);
        double y = Inches.convertFrom(startPose.getY(), inUnit);
        double r = Radians.convertFrom(startPose.getHeading(), angleUnit);
        return newTrajectory(new Pose2d(x, y, r));
    }

    /**
     * Create a new builder for a standard RoadRunner trajectory sequence using the drive system.
     * This will use the builder directly from RoadRunner, see {@link #addNewTrajectory(Pose2d)} for
     * integrated task building, or if you prefer you can use {@link #addTrajectory(TrajectorySequence)}
     * to add your sequence to the queue manually.
     *
     * @return Builder for the trajectory
     * @see #newTrajectorySequence(Pose2d)
     */
    @SuppressWarnings("rawtypes")
    default TrajectorySequenceBuilder newTrajectorySequence() {
        return getDrive().trajectorySequenceBuilder(getPreviousPose());
    }

    /**
     * Create a new builder for a standard RoadRunner trajectory sequence using the drive system.
     * This will use the builder directly from RoadRunner, see {@link #addNewTrajectory(Pose2d)} for
     * integrated task building, or if you prefer you can use {@link #addTrajectory(TrajectorySequence)}
     * to add your sequence to the queue manually.
     *
     * @param startPose Starting pose of the trajectory, ** WILL SET DRIVE POSE ESTIMATE TO THIS POSE **, (in, in, radians)
     * @return Builder for the trajectory
     */
    @SuppressWarnings("rawtypes")
    default TrajectorySequenceBuilder newTrajectorySequence(Pose2d startPose) {
        getDrive().setPoseEstimate(startPose);
        return getDrive().trajectorySequenceBuilder(startPose);
    }

    /**
     * Create a new builder for a standard RoadRunner trajectory sequence using the drive system.
     * This will use the builder directly from RoadRunner, see {@link #addNewTrajectory(Pose2d)} for
     * integrated task building, or if you prefer you can use {@link #addTrajectory(TrajectorySequence)}
     * to add your sequence to the queue manually.
     *
     * @param startPose Starting pose of the trajectory, ** WILL SET DRIVE POSE ESTIMATE TO THIS POSE **
     * @param inUnit    The unit of the end pose vector (will be converted to inches)
     * @param angleUnit The unit of the end pose heading (will be converted to radians)
     * @return Builder for the trajectory
     */
    @SuppressWarnings("rawtypes")
    default TrajectorySequenceBuilder newTrajectorySequence(Pose2d startPose, Distance inUnit, Angle angleUnit) {
        double x = Inches.convertFrom(startPose.getX(), inUnit);
        double y = Inches.convertFrom(startPose.getY(), inUnit);
        double r = Radians.convertFrom(startPose.getHeading(), angleUnit);
        return newTrajectorySequence(new Pose2d(x, y, r));
    }

    /**
     * Add a trajectory to the task queue with a timeout and priority level.
     *
     * @param trajectory Trajectory to add
     * @return Builder for the task
     */
    default AddTrajectoryBuilder<Trajectory> addTrajectory(Trajectory trajectory) {
        return new AddTrajectoryBuilder<>(trajectory, getDrive());
    }

    /**
     * Add a trajectory sequence to the task queue with a timeout and priority level.
     *
     * @param trajectory Trajectory sequence to add
     * @return Builder for the task
     */
    default AddTrajectoryBuilder<TrajectorySequence> addTrajectory(TrajectorySequence trajectory) {
        return new AddTrajectoryBuilder<>(trajectory, getDrive());
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
     * Builder class for adding a trajectory to the task queue. This will not work in a normal {@link BunyipsOpMode}.
     * This doesn't need to be used directly, as the {@link #addTrajectory(Trajectory)} method will handle this functionality.
     *
     * @param <S> Trajectory type
     */
    final class AddTrajectoryBuilder<S> {
        private final S trajectory;
        private final RoadRunnerDrive drive;
        private Measure<Time> timeout = DEFAULT_TIMEOUT;
        private PriorityLevel priority = PriorityLevel.NORMAL;
        private String name = null;

        /**
         * Create a new builder for adding a trajectory to the task queue.
         *
         * @param trajectory Trajectory to add
         * @param drive      Drive system to use
         */
        public AddTrajectoryBuilder(S trajectory, RoadRunnerDrive drive) {
            if (trajectory == null)
                throw new NullPointerException("trajectory cannot be null!");
            if (drive == null)
                throw new NullPointerException("drive cannot be null!");
            if (!BunyipsOpMode.isRunning() || !(BunyipsOpMode.getInstance() instanceof AutonomousBunyipsOpMode))
                throw new IllegalStateException("RoadRunner Task additions can only be used in an AutonomousBunyipsOpMode!");
            if (!(trajectory instanceof Trajectory) && !(trajectory instanceof TrajectorySequence))
                throw new EmergencyStop("trajectory must be a Trajectory or TrajectorySequence!");
            this.trajectory = trajectory;
            this.drive = drive;
        }

        /**
         * Set a timeout for the trajectory, to be applied to the overhead task running the trajectory.
         *
         * @param interval Timeout for the trajectory
         * @return trajectory builder
         */
        public AddTrajectoryBuilder<S> withTimeout(Measure<Time> interval) {
            timeout = interval;
            return this;
        }

        /**
         * Set the priority level of the task.
         *
         * @param p Priority level
         * @return trajectory builder
         */
        public AddTrajectoryBuilder<S> withPriority(PriorityLevel p) {
            priority = p;
            return this;
        }

        /**
         * Set the task name of the trajectory to show up in the telemetry.
         *
         * @param taskName Name of the task
         * @return trajectory builder
         */
        public AddTrajectoryBuilder<S> withName(String taskName) {
            name = taskName;
            return this;
        }

        /**
         * Add the trajectory to the task queue based on the builder arguments.
         */
        public void build() {
            RoadRunnerTask<RoadRunnerDrive> task = null;
            if (trajectory instanceof Trajectory) {
                task = new RoadRunnerTask<>(timeout, drive, (Trajectory) trajectory);
            } else if (trajectory instanceof TrajectorySequence) {
                task = new RoadRunnerTask<>(timeout, drive, (TrajectorySequence) trajectory);
            }
            assert task != null;
            task.withName(name);
            rrTasks.add(task);
            // We can assume we are in an AutonomousBunyipsOpMode as the constructor already checks for this
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

    /**
     * Builder class for a RoadRunner trajectory, which supports adding the trajectory to the Task queue.
     * This class may be used outside of an {@link AutonomousBunyipsOpMode} to build a trajectory sequence, but
     * cannot be used to add the sequence to the task queue.
     * This doesn't need to be used directly, as the {@link #newTrajectory()} method will handle this functionality.
     */
    final class RoadRunnerTrajectoryTaskBuilder extends TrajectorySequenceBuilder<RoadRunnerTrajectoryTaskBuilder> {
        private final RoadRunnerDrive drive;
        private Measure<Time> timeout = DEFAULT_TIMEOUT;
        private PriorityLevel priority = PriorityLevel.NORMAL;
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
            this.drive = drive;
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
         * Build the trajectory sequence without adding it to the task queue.
         * This method is useful if you are not running an {@link AutonomousBunyipsOpMode}.
         *
         * @return The built trajectory sequence from the builder
         */
        public TrajectorySequence buildOnlyTrajectory() {
            return super.build();
        }

        /**
         * Build the trajectory sequence and task, without adding it to a task queue.
         * This method is useful if you are not running an {@link AutonomousBunyipsOpMode}, but still want to
         * use the task system to run the trajectory with your own implementation.
         *
         * @return The built task.
         */
        public RoadRunnerTask<RoadRunnerDrive> buildOnlyTask() {
            TrajectorySequence sequence = super.build();
            RoadRunnerTask<RoadRunnerDrive> task = new RoadRunnerTask<>(timeout, drive, sequence);
            task.withTimeout(timeout);
            task.withName(name);
            return task;
        }

        /**
         * Build the trajectory sequence and add it to the task queue.
         * This method will only work in an {@link AutonomousBunyipsOpMode}.
         */
        @Override
        public TrajectorySequence build() {
            TrajectorySequence sequence = super.build();
            AddTrajectoryBuilder<TrajectorySequence> builder = new AddTrajectoryBuilder<>(sequence, drive);
            builder.withTimeout(timeout)
                    .withPriority(priority)
                    .withName(name)
                    .build();
            return sequence;
        }
    }
}
