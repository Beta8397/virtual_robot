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

import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceRunner;

import java.util.Arrays;
import java.util.List;

/**
 * Interface for access methods in RoadRunner drive classes.
 *
 * @author Lucas Bubner, 2023
 */
public interface RoadRunnerDrive {
    static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    TrajectorySequenceRunner getTrajectorySequenceRunner();

    void stop();

    void waitForIdle();

    DriveConstants getConstants();

    TrajectoryBuilder trajectoryBuilder(Pose2d startPose);

    TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed);

    TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading);

    @SuppressWarnings("rawtypes")
    TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose);

    void turnAsync(double angle);

    void turn(double angle);

    void followTrajectoryAsync(Trajectory trajectory);

    void followTrajectory(Trajectory trajectory);

    void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence);

    void followTrajectorySequence(TrajectorySequence trajectorySequence);

    Pose2d getLastError();

    void update();

    boolean isBusy();

    void cancelTrajectory();

    void setMode(DcMotor.RunMode runMode);

    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior);

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

    void updatePoseEstimate();

    void setDriveSignal(DriveSignal driveSignal);

    void setDrivePower(Pose2d drivePower);

    void setWeightedDrivePowerFieldCentric(Pose2d pose);
}
