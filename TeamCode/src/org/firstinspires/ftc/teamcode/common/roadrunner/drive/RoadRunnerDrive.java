package org.firstinspires.ftc.teamcode.common.roadrunner.drive;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.common.roadrunner.trajectorysequence.TrajectorySequence;

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

    TrajectoryBuilder trajectoryBuilder(Pose2d startPose);

    TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed);

    TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading);

    void turnAsync(double angle);

    void turn(double angle);

    void followTrajectoryAsync(Trajectory trajectory);

    void followTrajectory(Trajectory trajectory);

    void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence);

    void followTrajectorySequence(TrajectorySequence trajectorySequence);

    Pose2d getLastError();

    // waitForIdle was removed as BunyipsOpMode handles all dispatch manually
    void update();

    boolean isBusy();

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
}
