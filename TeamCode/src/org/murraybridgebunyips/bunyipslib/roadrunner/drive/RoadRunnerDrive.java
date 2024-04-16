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
     * Get a velocity constraint for the drive.
     *
     * @param maxVel        The maximum velocity of the drive.
     * @param maxAngularVel The maximum angular velocity of the drive.
     * @param trackWidth    The track width of the drive.
     * @return A velocity constraint for the drive.
     */
    static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    /**
     * Get an acceleration constraint for the drive.
     *
     * @param maxAccel The maximum acceleration of the drive.
     * @return An acceleration constraint for the drive.
     */
    static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    TrajectorySequenceRunner getTrajectorySequenceRunner();

    /**
     * Stop the drive.
     */
    void stop();

    /**
     * Run current trajectory blocking until it is complete.
     */
    void waitForIdle();

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
     * Get a trajectory builder for the drive.
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
     * Turn the drive.
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

    Pose2d getLastError();

    /**
     * Update the drive with latest motor powers.
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

    void setMode(DcMotor.RunMode runMode);

    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior);

    /**
     * Set the PIDF coefficients for the drive motors.
     *
     * @param runMode      The run mode of the drive.
     * @param coefficients The PIDF coefficients for the motors.
     */
    void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients);

    void setWeightedDrivePower(Pose2d drivePower);

    List<Double> getWheelPositions();

    List<Double> getWheelVelocities();

    // Must be implemented manually due to different numbers of motors
    // void setMotorPowers(...);

    double getRawExternalHeading();

    Double getExternalHeadingVelocity();

    Localizer getLocalizer();

    void setLocalizer(Localizer localizer);

    double getExternalHeading();

    void setExternalHeading(double value);

    Pose2d getPoseEstimate();

    void setPoseEstimate(Pose2d value);

    Pose2d getPoseVelocity();

    /**
     * Update the pose estimate of the drive.
     */
    void updatePoseEstimate();

    void setDriveSignal(DriveSignal driveSignal);

    void setDrivePower(Pose2d drivePower);

    void setWeightedDrivePowerFieldCentric(Pose2d pose);
}
