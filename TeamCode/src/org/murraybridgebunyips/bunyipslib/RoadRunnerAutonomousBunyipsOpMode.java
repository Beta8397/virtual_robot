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

/**
 * RoadRunnerAutonomousBunyipsOpMode (RRABOM, nickname "Rabone")
 * Additional abstraction for RoadRunner drives to integrate trajectories seamlessly into Autonomous.
 *
 * @author Lucas Bubner, 2023
 */
public abstract class RoadRunnerAutonomousBunyipsOpMode<T extends RoadRunnerDrive> extends AutonomousBunyipsOpMode {

    /**
     * Default timeout for RoadRunner tasks, a value of 0.0 will run the task until it is finished.
     */
    public static final double DEFAULT_TIMEOUT = 0.0;

    /**
     * Drive instance to be used for RoadRunner trajectories.
     * This is automatically set by the {@link #setDrive()} method, ensuring that the drive instance
     * is set before any tasks are added to the queue.
     * <p>
     * {@code drive = new MecanumDrive(...)}
     */
    protected T drive;

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
    protected final void onInit() {
        super.onInit();
        if (drive == null)
            drive = setDrive();
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
    public RoadRunnerTrajectoryTaskBuilder addNewTrajectory(Pose2d startPose) {
        if (drive == null)
            drive = setDrive();
        if (drive == null) throw new NullPointerException("drive instance is not set!");
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
     * Without arguments, will use the current pose estimate of the drive.
     *
     * @return Builder for the trajectory
     * @see #addNewTrajectory(Pose2d)
     */
    public RoadRunnerTrajectoryTaskBuilder addNewTrajectory() {
        if (drive == null)
            drive = setDrive();
        if (drive == null) throw new NullPointerException("drive instance is not set!");
        // noinspection rawtypes
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(drive.getPoseEstimate());
        return new RoadRunnerTrajectoryTaskBuilder(drive.getPoseEstimate(), builder.getBaseVelConstraint(), builder.getBaseAccelConstraint(), builder.getBaseTurnConstraintMaxAngVel(), builder.getBaseTurnConstraintMaxAngAccel());
    }

    // Internal method to get the OpMode instance from an inner class
    private RoadRunnerAutonomousBunyipsOpMode<T> getOpMode() {
        return this;
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
    public TrajectoryBuilder newTrajectory() {
        if (drive == null)
            drive = setDrive();
        if (drive == null) throw new NullPointerException("drive instance is not set!");
        return drive.trajectoryBuilder(drive.getPoseEstimate());
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
    public TrajectoryBuilder newTrajectory(Pose2d startPose) {
        if (drive == null)
            drive = setDrive();
        if (drive == null) throw new NullPointerException("drive instance is not set!");
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
    public TrajectorySequenceBuilder newTrajectorySequence() {
        if (drive == null)
            drive = setDrive();
        if (drive == null) throw new NullPointerException("drive instance is not set!");
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate());
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
    public TrajectorySequenceBuilder newTrajectorySequence(Pose2d startPose) {
        if (drive == null)
            drive = setDrive();
        if (drive == null) throw new NullPointerException("drive instance is not set!");
        drive.setPoseEstimate(startPose);
        return drive.trajectorySequenceBuilder(startPose);
    }

    /**
     * Add a RoadRunner trajectory to the queue, with a default infinite timeout and default priority.
     * To customise the timeout, see {@link #addTrajectory(Trajectory, double)}.
     * To customise the priority, see {@link #addTrajectory(Trajectory, PriorityLevel)}.
     * To customise both, see {@link #addTrajectory(Trajectory, PriorityLevel, double)}.
     *
     * @param trajectory Trajectory to add
     */
    public void addTrajectory(Trajectory trajectory) {
        if (drive == null)
            drive = setDrive();
        if (drive == null) throw new NullPointerException("drive instance is not set!");
        addTask(new RoadRunnerTask<>(this, DEFAULT_TIMEOUT, drive, trajectory));
    }

    /**
     * Add a RoadRunner trajectory to the queue, with a default infinite timeout.
     * To customise the timeout, see {@link #addTrajectory(Trajectory, PriorityLevel, double)}.
     *
     * @param trajectory Trajectory to add
     * @param priority   Priority level of the task, see {@link PriorityLevel}
     */
    public void addTrajectory(Trajectory trajectory, PriorityLevel priority) {
        if (drive == null)
            drive = setDrive();
        if (drive == null) throw new NullPointerException("drive instance is not set!");
        switch (priority) {
            case LAST:
                addTaskLast(new RoadRunnerTask<>(this, DEFAULT_TIMEOUT, drive, trajectory));
                break;
            case NORMAL:
                addTask(new RoadRunnerTask<>(this, DEFAULT_TIMEOUT, drive, trajectory));
                break;
            case FIRST:
                addTaskFirst(new RoadRunnerTask<>(this, DEFAULT_TIMEOUT, drive, trajectory));
                break;
        }
    }

    /**
     * Add a RoadRunner trajectory to the queue, with a task timeout other than the default, and default priority.
     * To customise the priority, see {@link #addTrajectory(Trajectory, PriorityLevel, double)}.
     *
     * @param trajectory Trajectory to add
     * @param timeout    Timeout in seconds
     */
    public void addTrajectory(Trajectory trajectory, double timeout) {
        if (drive == null)
            drive = setDrive();
        if (drive == null) throw new NullPointerException("drive instance is not set!");
        addTask(new RoadRunnerTask<>(this, timeout, drive, trajectory));
    }

    /**
     * Add a RoadRunner trajectory to the queue, with a task timeout other than the default.
     *
     * @param trajectory Trajectory to add
     * @param timeout    Timeout in seconds
     * @param priority   Priority level of the task, see {@link PriorityLevel}
     */
    public void addTrajectory(Trajectory trajectory, PriorityLevel priority, double timeout) {
        if (drive == null)
            drive = setDrive();
        if (drive == null) throw new NullPointerException("drive instance is not set!");
        switch (priority) {
            case LAST:
                addTaskLast(new RoadRunnerTask<>(this, timeout, drive, trajectory));
                break;
            case NORMAL:
                addTask(new RoadRunnerTask<>(this, timeout, drive, trajectory));
                break;
            case FIRST:
                addTaskFirst(new RoadRunnerTask<>(this, timeout, drive, trajectory));
                break;
        }
    }

    /**
     * Add a RoadRunner trajectory to the queue, with a default infinite timeout and default priority.
     * To customise the timeout, see {@link #addTrajectory(TrajectorySequence, double)}.
     * To customise the priority, see {@link #addTrajectory(TrajectorySequence, PriorityLevel)}.
     * To customise both, see {@link #addTrajectory(TrajectorySequence, PriorityLevel, double)}.
     *
     * @param trajectorySequence Trajectory to add
     */
    public void addTrajectory(TrajectorySequence trajectorySequence) {
        if (drive == null)
            drive = setDrive();
        if (drive == null) throw new NullPointerException("drive instance is not set!");
        addTask(new RoadRunnerTask<>(this, DEFAULT_TIMEOUT, drive, trajectorySequence));
    }

    /**
     * Add a RoadRunner trajectory to the queue, with a default infinite timeout.
     * To customise the timeout, see {@link #addTrajectory(TrajectorySequence, PriorityLevel, double)}.
     *
     * @param trajectorySequence Trajectory to add
     * @param priority           Priority level of the task, see {@link PriorityLevel}
     */
    public void addTrajectory(TrajectorySequence trajectorySequence, PriorityLevel priority) {
        if (drive == null)
            drive = setDrive();
        if (drive == null) throw new NullPointerException("drive instance is not set!");
        switch (priority) {
            case LAST:
                addTaskLast(new RoadRunnerTask<>(this, DEFAULT_TIMEOUT, drive, trajectorySequence));
                break;
            case NORMAL:
                addTask(new RoadRunnerTask<>(this, DEFAULT_TIMEOUT, drive, trajectorySequence));
                break;
            case FIRST:
                addTaskFirst(new RoadRunnerTask<>(this, DEFAULT_TIMEOUT, drive, trajectorySequence));
                break;
        }
    }

    /**
     * Add a RoadRunner trajectory to the queue, with a task timeout other than the default, and default priority.
     * To customise the timeout, see {@link #addTrajectory(TrajectorySequence, PriorityLevel, double)}.
     *
     * @param trajectorySequence Trajectory to add
     * @param timeout            Timeout in seconds
     */
    public void addTrajectory(TrajectorySequence trajectorySequence, double timeout) {
        if (drive == null)
            drive = setDrive();
        if (drive == null) throw new NullPointerException("drive instance is not set!");
        addTask(new RoadRunnerTask<>(this, timeout, drive, trajectorySequence));
    }

    /**
     * Add a RoadRunner trajectory to the queue, with a task timeout other than the default.
     *
     * @param trajectorySequence Trajectory to add
     * @param timeout            Timeout in seconds
     * @param priority           Priority level of the task, see {@link PriorityLevel}
     */
    public void addTrajectory(TrajectorySequence trajectorySequence, PriorityLevel priority, double timeout) {
        if (drive == null)
            drive = setDrive();
        if (drive == null) throw new NullPointerException("drive instance is not set!");
        switch (priority) {
            case LAST:
                addTaskLast(new RoadRunnerTask<>(this, timeout, drive, trajectorySequence));
                break;
            case NORMAL:
                addTask(new RoadRunnerTask<>(this, timeout, drive, trajectorySequence));
                break;
            case FIRST:
                addTaskFirst(new RoadRunnerTask<>(this, timeout, drive, trajectorySequence));
                break;
        }
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
        FIRST // (tech challenge)
    }

    /**
     * Builder class for a RoadRunner trajectory, which supports adding the trajectory to the Task queue.
     */
    protected class RoadRunnerTrajectoryTaskBuilder extends TrajectorySequenceBuilder<RoadRunnerTrajectoryTaskBuilder> {
        private double timeout = DEFAULT_TIMEOUT;

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
         * Build the trajectory sequence and add it to the task queue with default priority.
         */
        @Override
        public TrajectorySequence build() {
            if (drive == null)
                drive = setDrive();
            if (drive == null) throw new NullPointerException("drive instance is not set!");
            TrajectorySequence builtTrajectory = super.build();
            addTask(new RoadRunnerTask<>(getOpMode(), timeout, drive, builtTrajectory));
            return builtTrajectory;
        }

        /**
         * Optional method to build as the very first thing the robot will do.
         *
         * @see PriorityLevel
         */
        public TrajectorySequence buildWithPriority() {
            if (drive == null)
                drive = setDrive();
            if (drive == null) throw new NullPointerException("drive instance is not set!");
            TrajectorySequence builtTrajectory = super.build();
            addTaskFirst(new RoadRunnerTask<>(getOpMode(), timeout, drive, builtTrajectory));
            return builtTrajectory;
        }

        /**
         * Optional method to build as the very last thing the robot will do.
         *
         * @see PriorityLevel
         */
        public TrajectorySequence buildWithLowPriority() {
            if (drive == null)
                drive = setDrive();
            if (drive == null) throw new NullPointerException("drive instance is not set!");
            TrajectorySequence builtTrajectory = super.build();
            addTaskLast(new RoadRunnerTask<>(getOpMode(), timeout, drive, builtTrajectory));
            return builtTrajectory;
        }
    }
}
