package org.murraybridgebunyips.bunyipslib.roadrunner.drive;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.murraybridgebunyips.bunyipslib.Storage;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.drive.TankDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceRunner;

import java.util.Arrays;
import java.util.List;

/**
 * Interface for access methods in RoadRunner drive classes, such as {@link MecanumDrive} and {@link TankDrive}.
 *
 * @author Lucas Bubner, 2023
 */
public interface RoadRunnerDrive {
    /**
     * Set a remembered pose from memory in another OpMode.
     */
    default void updatePoseFromMemory() {
        // Important to set this after the localizer is configured, as the localizer change will reset the pose estimate
        if (Storage.memory().lastKnownPosition != null) {
            setPoseEstimate(Storage.memory().lastKnownPosition);
        }
    }

    /**
     * Get a velocity constraint for the drive.
     * Override this method to use a custom velocity constraint.
     *
     * @param maxVel        The maximum velocity of the drive.
     * @param maxAngularVel The maximum angular velocity of the drive.
     * @param trackWidth    The track width of the drive.
     * @return A velocity constraint for the drive.
     */
    default TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    /**
     * Get an acceleration constraint for the drive.
     * Override this method to use a custom acceleration constraint.
     *
     * @param maxAccel The maximum acceleration of the drive.
     * @return An acceleration constraint for the drive.
     */
    default TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    /**
     * Get the attached TrajectorySequenceRunner for this drive.
     *
     * @return the trajectory sequence runner in use
     */
    TrajectorySequenceRunner getTrajectorySequenceRunner();

    /**
     * Stop the drive.
     */
    void stop();

    /**
     * Run current trajectory blocking until it is complete. Usually not recommended as it is fully blocking.
     */
    void waitForIdle();

    /**
     * Get the drive constants related to this drive.
     *
     * @return the drive constants used in constructing this drive
     */
    DriveConstants getConstants();

    /**
     * Get a trajectory builder for the drive.
     *
     * @param startPose The starting pose of the drive.
     * @return A trajectory builder for the drive.
     */
    TrajectoryBuilder trajectoryBuilder(Pose2d startPose);

    /**
     * Get a trajectory builder for the drive.
     *
     * @param startPose The starting pose of the drive.
     * @param reversed  Whether the drive is reversed.
     * @return A trajectory builder for the drive.
     */
    TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed);

    /**
     * Get a trajectory builder for the drive.
     *
     * @param startPose    The starting pose of the drive.
     * @param startHeading The starting heading of the drive.
     * @return A trajectory builder for the drive.
     */
    TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading);

    /**
     * Get a trajectory sequence builder for the drive.
     *
     * @param startPose The starting pose of the drive.
     * @return A trajectory builder for the drive.
     */
    @SuppressWarnings("rawtypes")
    TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose);

    /**
     * Turn the drive asynchronously.
     *
     * @param angle The angle to turn the drive in radians.
     */
    void turnAsync(double angle);

    /**
     * Turn the drive. Note this is blocking.
     *
     * @param angle The angle to turn the drive in radians.
     */
    void turn(double angle);

    /**
     * Follow a trajectory asynchronously.
     *
     * @param trajectory The trajectory to follow.
     */
    void followTrajectoryAsync(Trajectory trajectory);

    /**
     * Follow a trajectory.
     *
     * @param trajectory The trajectory to follow.
     */
    void followTrajectory(Trajectory trajectory);

    /**
     * Follow a trajectory sequence asynchronously.
     *
     * @param trajectorySequence The trajectory sequence to follow.
     */
    void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence);

    /**
     * Follow a trajectory sequence.
     *
     * @param trajectorySequence The trajectory sequence to follow.
     */
    void followTrajectorySequence(TrajectorySequence trajectorySequence);

    /**
     * Get the last reported pose error from the drive.
     *
     * @return odometry pose error as reported last update
     */
    Pose2d getLastError();

    /**
     * Update the drive with latest motor powers and odometry.
     */
    void update();

    /**
     * @return Whether the drive is busy running a trajectory.
     */
    boolean isBusy();

    /**
     * Abort the current trajectory.
     */
    void cancelTrajectory();

    /**
     * Set the run mode of all motors.
     *
     * @param runMode new run mode to apply to all motors on this drive.
     */
    void setMode(DcMotor.RunMode runMode);

    /**
     * Set the zero power behaviour of all motors.
     *
     * @param zeroPowerBehavior new zero power behaviour to apply to all motors on this drive.
     */
    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior);

    /**
     * Set the PIDF coefficients for the drive motors.
     *
     * @param runMode      The run mode of the drive.
     * @param coefficients The PIDF coefficients for the motors.
     */
    void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients);

    /**
     * Set a drive power with axial weights. Feedforward is not applied.
     *
     * @param drivePower the robot-centric pose to travel in.
     */
    void setWeightedDrivePower(Pose2d drivePower);

    /**
     * @return the encoder report of wheel positions
     */
    List<Double> getWheelPositions();

    /**
     * @return the encoder report of wheel velocities
     */
    List<Double> getWheelVelocities();

    // Must be implemented manually due to different numbers of motors
    // void setMotorPowers(...);

    /**
     * @return the current heading as dictated by an external source (gyro, etc)
     */
    double getRawExternalHeading();

    /**
     * @return the current heading velocity as dictated by an external source (gyro, etc)
     */
    Double getExternalHeadingVelocity();

    /**
     * @return the localizer in use by this drive
     */
    Localizer getLocalizer();

    /**
     * Set the new localizer to use. Note that pose estimates from the old localizer may be discarded.
     *
     * @param localizer the new localizer
     */
    void setLocalizer(Localizer localizer);

    /**
     * @return the current heading of the robot
     */
    double getExternalHeading();

    /**
     * Assert the heading of the robot.
     *
     * @param value the new heading that should be considered as the current heading
     */
    void setExternalHeading(double value);

    /**
     * @return the currently calculated pose from the localizer of where the robot is on the field
     */
    Pose2d getPoseEstimate();

    /**
     * Assert the position of the robot.
     *
     * @param value the new pose that should be considered the current pose
     */
    void setPoseEstimate(Pose2d value);

    /**
     * @return the change of pose (pose delta since last update)
     */
    Pose2d getPoseVelocity();

    /**
     * Update the pose estimate of the drive.
     */
    void updatePoseEstimate();

    /**
     * Set a drive velocity and/or acceleration target. Feedforward is applied.
     *
     * @param driveSignal the velocity and acceleration that should be commanded
     */
    void setDriveSignal(DriveSignal driveSignal);

    /**
     * Set a drive power. Feedforward and axial weights are not applied.
     *
     * @param drivePower the robot-centric pose to apply to the drive
     */
    void setDrivePower(Pose2d drivePower);

    /**
     * Set a drive power with axial weights and in a field-centric frame of reference.
     *
     * @param pose the robot-centric pose to apply to the drive, will be rotated to be field-centric internally
     */
    void setWeightedDrivePowerFieldCentric(Pose2d pose);
}
