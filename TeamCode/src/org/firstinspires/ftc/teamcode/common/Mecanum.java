package org.firstinspires.ftc.teamcode.common;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.Drive;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.MecanumCoefficients;
import org.firstinspires.ftc.teamcode.common.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;

/**
 * Wrapper component for the RoadRunner Mecanum Drive, bringing RoadRunner functionality
 * to the BunyipsOpMode framework.
 *
 * @author Lucas Bubner, 2023
 */
public class Mecanum extends BunyipsComponent implements Drive {
    private final org.firstinspires.ftc.teamcode.common.roadrunner.drive.MecanumDrive drive;

    public Mecanum(@NonNull BunyipsOpMode opMode, DriveConstants constants, MecanumCoefficients mecanumCoefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, IMU imu, DcMotorEx frontLeft, DcMotorEx backLeft, DcMotorEx frontRight, DcMotorEx backRight) {
        super(opMode);
        drive = new org.firstinspires.ftc.teamcode.common.roadrunner.drive.MecanumDrive(constants, mecanumCoefficients, voltageSensor, imu, frontLeft, frontRight, backLeft, backRight);
    }

    public org.firstinspires.ftc.teamcode.common.roadrunner.drive.MecanumDrive getInstance() {
        return drive;
    }

    @Override
    public double getExternalHeading() {
        return drive.getExternalHeading();
    }

    @Override
    public void setExternalHeading(double value) {
        drive.setExternalHeading(value);
    }

    public void setPowers(double v, double v1, double v2, double v3) {
        drive.setMotorPowers(v, v1, v2, v3);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }

    @Override
    public void turnAsync(double angle) {
        drive.turnAsync(angle);
    }

    @Override
    public void turn(double angle) {
        drive.turn(angle);
    }

    @Override
    public void followTrajectoryAsync(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    @Override
    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectory(trajectory);
    }

    @Override
    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequenceAsync(trajectorySequence);
    }

    @Override
    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequence(trajectorySequence);
    }

    @Override
    public Pose2d getLastError() {
        return drive.getLastError();
    }

    @Override
    public void update() {
        drive.update();
    }

    @Override
    public boolean isBusy() {
        return drive.isBusy();
    }

    @Override
    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    @Override
    public void setPoseEstimate(Pose2d value) {
        drive.setPoseEstimate(value);
    }

    @Override
    public Pose2d getPoseVelocity() {
        return drive.getPoseVelocity();
    }

    @Override
    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    @Override
    public void setDriveSignal(DriveSignal driveSignal) {
        drive.setDriveSignal(driveSignal);
    }

    @Override
    public void setDrivePower(Pose2d drivePower) {
        drive.setDrivePower(drivePower);
    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {
        drive.setMode(runMode);
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        drive.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(runMode, coefficients);
    }

    @Override
    public void setWeightedDrivePower(Pose2d drivePower) {
        drive.setWeightedDrivePower(drivePower);
    }

    @Override
    public List<Double> getWheelPositions() {
        return drive.getWheelPositions();
    }

    @Override
    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }

    @Override
    public double getRawExternalHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @Override
    public Localizer getLocalizer() {
        return drive.getLocalizer();
    }

    public void setLocalizer(Localizer localizer) {
        drive.setLocalizer(localizer);
    }
}