package org.murraybridgebunyips.bunyipslib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.murraybridgebunyips.bunyipslib.tasks.RoadRunnerTask;

import java.util.ArrayList;

/**
 * RoadRunnerAutonomousBunyipsOpMode (RRABOM, nickname "Rabone").
 * Superset of {@link AutonomousBunyipsOpMode} that integrates RoadRunner trajectories into the task queue through
 * utility methods such as {@code addNewTrajectory}.
 *
 * @param <T> RoadRunner drive instance
 * @author Lucas Bubner, 2023
 * @see BunyipsOpMode
 */
public abstract class RoadRunnerAutonomousBunyipsOpMode<T extends RoadRunnerDrive> extends AutonomousBunyipsOpMode {
    /**
     * Default timeout for RoadRunner tasks, a value of 0.0 will run the task until it is finished.
     */
    public static final double DEFAULT_TIMEOUT = 0.0;
    private final ArrayList<RoadRunnerTask<T>> rrTasks = new ArrayList<>();

    /**
     * Drive instance to be used for RoadRunner trajectories.
     * This is automatically set by the {@link #setDrive()} method, ensuring that the drive instance
     * is set before any tasks are added to the queue.
     * <p>
     * {@code drive = new MecanumDrive(...)}
     */
    protected T drive;

    /**
     * Runs upon the pressing of the INIT button on the Driver Station.
     * This is where your hardware should be initialised. You may also add specific tasks to the queue
     * here, but it is recommended to use {@link #setInitTask()} or {@link #onQueueReady(OpModeSelection)} instead.
     */
    protected abstract void onInitialise();

    /**
     * Set the drive instance to be used for RoadRunner trajectories. This method ensures
     * that the drive instance is set to avoid accidental NullPointerExceptions, similar to how
     * setOpModes and setInitTask are handled. This is called after initialisation, so your config
     * instance will be available.
     *
     * @return RoadRunner drive instance, can be instantiated here or as a class member
     */
    protected abstract T setDrive();

    @Override
    protected final void onInitialisation() {
        onInitialise();
        assertDrive();
    }

    private void assertDrive() {
        if (drive != null) return;
        drive = setDrive();
        if (drive == null)
            throw new NullPointerException("drive instance is not set!");
    }

    private Pose2d getPreviousPose() {
        // Needed to splice the last pose from the last trajectory
        return rrTasks.isEmpty() ? drive.getPoseEstimate() : rrTasks.get(rrTasks.size() - 1).getEndPose();
    }

    /**
     * STRONGLY RECOMMENDED: Use this method to build a new RoadRunner trajectory to the queue.
     * Creates a new builder for a RoadRunner trajectory, which will automatically add a
     * task to the queue when build() is called, optionally with a timeout control ({@link RoadRunnerTrajectoryTaskBuilder#setTimeout(double)}).
     * <p>
     * This method is the combination of {@link #newTrajectorySequence()} and {@link #addTrajectory(TrajectorySequence)},
     * using a custom builder that supports {@code setTimeout()} and priority building.
     *
     * @param startPose Starting pose of the trajectory, ** WILL SET DRIVE POSE ESTIMATE TO THIS POSE **
     * @return Builder for the trajectory
     */
    protected RoadRunnerTrajectoryTaskBuilder addNewTrajectory(Pose2d startPose) {
        assertDrive();
        // noinspection rawtypes
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
        drive.setPoseEstimate(startPose);
        return new RoadRunnerTrajectoryTaskBuilder(startPose, builder.getBaseVelConstraint(), builder.getBaseAccelConstraint(), builder.getBaseTurnConstraintMaxAngVel(), builder.getBaseTurnConstraintMaxAngAccel());
    }

    /**
     * STRONGLY RECOMMENDED: Use this method to build a new RoadRunner trajectory to the queue.
     * Creates a new builder for a RoadRunner trajectory, which will automatically add a
     * task to the queue when build() is called, optionally with a timeout control.
     * This method is the combination of {@link #newTrajectorySequence()} and {@link #addTrajectory(TrajectorySequence)},
     * using a custom builder that supports {@code setTimeout()} and priority building.
     * Without arguments, will use the current pose estimate of the drive or the last spliced pose.
     *
     * @return Builder for the trajectory
     * @see #addNewTrajectory(Pose2d)
     */
    protected RoadRunnerTrajectoryTaskBuilder addNewTrajectory() {
        assertDrive();
        // noinspection rawtypes
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(getPreviousPose());
        return new RoadRunnerTrajectoryTaskBuilder(getPreviousPose(), builder.getBaseVelConstraint(), builder.getBaseAccelConstraint(), builder.getBaseTurnConstraintMaxAngVel(), builder.getBaseTurnConstraintMaxAngAccel());
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
    protected TrajectoryBuilder newTrajectory() {
        assertDrive();
        return drive.trajectoryBuilder(getPreviousPose());
    }

    /**
     * Create a new builder for a standard RoadRunner trajectory using the drive system.
     * This will use the builder directly from RoadRunner, see {@link #addNewTrajectory(Pose2d)} for
     * integrated task building, or if you prefer you can use {@link #addTrajectory(Trajectory)}
     * to add your sequence to the queue manually.
     *
     * @param startPose Starting pose of the trajectory, ** WILL SET DRIVE POSE ESTIMATE TO THIS POSE **
     * @return Builder for the trajectory
     */
    protected TrajectoryBuilder newTrajectory(Pose2d startPose) {
        assertDrive();
        drive.setPoseEstimate(startPose);
        return drive.trajectoryBuilder(startPose);
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
    protected TrajectorySequenceBuilder newTrajectorySequence() {
        assertDrive();
        return drive.trajectorySequenceBuilder(getPreviousPose());
    }

    /**
     * Create a new builder for a standard RoadRunner trajectory sequence using the drive system.
     * This will use the builder directly from RoadRunner, see {@link #addNewTrajectory(Pose2d)} for
     * integrated task building, or if you prefer you can use {@link #addTrajectory(TrajectorySequence)}
     * to add your sequence to the queue manually.
     *
     * @param startPose Starting pose of the trajectory, ** WILL SET DRIVE POSE ESTIMATE TO THIS POSE **
     * @return Builder for the trajectory
     */
    @SuppressWarnings("rawtypes")
    protected TrajectorySequenceBuilder newTrajectorySequence(Pose2d startPose) {
        assertDrive();
        drive.setPoseEstimate(startPose);
        return drive.trajectorySequenceBuilder(startPose);
    }

    /**
     * Add a trajectory to the task queue with a timeout and priority level.
     *
     * @param trajectory Trajectory to add
     * @return Builder for the task
     */
    protected AddTrajectoryBuilder<Trajectory> addTrajectory(Trajectory trajectory) {
        return new AddTrajectoryBuilder<>(trajectory);
    }

    /**
     * Add a trajectory sequence to the task queue with a timeout and priority level.
     *
     * @param trajectory Trajectory sequence to add
     * @return Builder for the task
     */
    protected AddTrajectoryBuilder<TrajectorySequence> addTrajectory(TrajectorySequence trajectory) {
        return new AddTrajectoryBuilder<>(trajectory);
    }

    /**
     * Priority representation for building tasks.
     * LAST: Add the task to the end of the queue after the onQueueReady() init callback has fired
     * NORMAL: Add the task to the queue immediately (default)
     * FIRST: Add the task to the front of the queue after the onQueueReady() init callback has fired
     */
    protected enum PriorityLevel {
        LAST,
        NORMAL,
        FIRST//® Tech Challenge
    }

    protected class AddTrajectoryBuilder<S> {
        private final S trajectory;
        private double timeout = DEFAULT_TIMEOUT;
        private PriorityLevel priority = PriorityLevel.NORMAL;
        private String name = null;

        public AddTrajectoryBuilder(S trajectory) {
            if (trajectory == null)
                throw new NullPointerException("trajectory cannot be null!");
            if (!(trajectory instanceof Trajectory) && !(trajectory instanceof TrajectorySequence))
                throw new EmergencyStop("trajectory must be a Trajectory or TrajectorySequence!");
            this.trajectory = trajectory;
        }

        public AddTrajectoryBuilder<S> withTimeout(double sec) {
            timeout = Math.abs(sec);
            return this;
        }

        public AddTrajectoryBuilder<S> withPriority(PriorityLevel p) {
            priority = p;
            return this;
        }

        public AddTrajectoryBuilder<S> withName(String taskName) {
            name = taskName;
            return this;
        }

        /**
         * Add the trajectory to the task queue based on the builder arguments.
         */
        public void build() {
            assertDrive();
            RoadRunnerTask<T> task = null;
            if (trajectory instanceof Trajectory) {
                task = new RoadRunnerTask<>(timeout, drive, (Trajectory) trajectory);
            } else if (trajectory instanceof TrajectorySequence) {
                task = new RoadRunnerTask<>(timeout, drive, (TrajectorySequence) trajectory);
            }
            assert task != null;
            task.withName(name);
            rrTasks.add(task);
            switch (priority) {
                case LAST:
                    addTaskLast(task);
                    break;
                case NORMAL:
                    addTask(task);
                    break;
                case FIRST:
                    addTaskFirst(task);
                    break;
            }
        }
    }

    /**
     * Builder class for a RoadRunner trajectory, which supports adding the trajectory to the Task queue.
     */
    protected class RoadRunnerTrajectoryTaskBuilder extends TrajectorySequenceBuilder<RoadRunnerTrajectoryTaskBuilder> {
        private double timeout = DEFAULT_TIMEOUT;
        private String name = null;

        public RoadRunnerTrajectoryTaskBuilder(Pose2d startPose, Double startTangent, TrajectoryVelocityConstraint baseVelConstraint, TrajectoryAccelerationConstraint baseAccelConstraint, double baseTurnConstraintMaxAngVel, double baseTurnConstraintMaxAngAccel) {
            super(startPose, startTangent, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
        }

        public RoadRunnerTrajectoryTaskBuilder(Pose2d startPose, TrajectoryVelocityConstraint baseVelConstraint, TrajectoryAccelerationConstraint baseAccelConstraint, double baseTurnConstraintMaxAngVel, double baseTurnConstraintMaxAngAccel) {
            super(startPose, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
        }

        /**
         * Set a timeout for the trajectory, to be applied to the overhead task running the trajectory.
         * Should be called first, before any other builder methods.
         * If this method is not called or fed negative values, an infinite timeout will be used.
         *
         * @param timeout Timeout in seconds
         * @return trajectory builder
         */
        // javascript reference incoming
        public RoadRunnerTrajectoryTaskBuilder setTimeout(double timeout) {
            // javascript reference is done
            if (timeout < 0) {
                return this;
            }
            this.timeout = timeout;
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
         * Build the trajectory sequence and add it to the task queue with default priority.
         */
        @Override
        public TrajectorySequence build() {
            assertDrive();
            TrajectorySequence builtTrajectory = super.build();
            addTask(createTask(builtTrajectory));
            return builtTrajectory;
        }

        /**
         * Optional method to build as the very first thing the robot will do.
         *
         * @see PriorityLevel
         */
        public TrajectorySequence buildWithPriority() {
            assertDrive();
            TrajectorySequence builtTrajectory = super.build();
            addTaskFirst(createTask(builtTrajectory));
            return builtTrajectory;
        }

        /**
         * Optional method to build as the very last thing the robot will do.
         *
         * @see PriorityLevel
         */
        public TrajectorySequence buildWithLowPriority() {
            assertDrive();
            TrajectorySequence builtTrajectory = super.build();
            addTaskLast(createTask(builtTrajectory));
            return builtTrajectory;
        }

        private RoadRunnerTask<T> createTask(TrajectorySequence builtTrajectory) {
            RoadRunnerTask<T> task = new RoadRunnerTask<>(timeout, drive, builtTrajectory);
            task.withName(name);
            rrTasks.add(task);
            return task;
        }
    }
}
