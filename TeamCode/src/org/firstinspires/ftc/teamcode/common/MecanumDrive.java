package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.teamcode.common.Text.round;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.MecanumCoefficients;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.MecanumRoadRunnerDrive;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.RoadRunnerDrive;
import org.firstinspires.ftc.teamcode.common.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.common.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.common.roadrunner.trajectorysequence.TrajectorySequenceRunner;

import java.util.List;

/**
 * Wrapper component for the RoadRunner Mecanum Drive, bringing RoadRunner functionality
 * to the BunyipsOpMode framework.
 *
 * @author Lucas Bubner, 2023
 */
public class MecanumDrive extends BunyipsComponent implements RoadRunnerDrive {
    private final MecanumRoadRunnerDrive drive;
    private final IMU imu;

    public MecanumDrive(@NonNull BunyipsOpMode opMode, DriveConstants constants, MecanumCoefficients mecanumCoefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, IMU imu, DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br) {
        super(opMode);
        drive = new MecanumRoadRunnerDrive(constants, mecanumCoefficients, voltageSensor, imu, fl, fr, bl, br);
        if (RobotConfig.getLastKnownPosition() != null) {
            drive.setPoseEstimate(RobotConfig.getLastKnownPosition());
        }
        this.imu = imu;
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    @Override
    public TrajectorySequenceRunner getTrajectorySequenceRunner() {
        return drive.getTrajectorySequenceRunner();
    }

    /**
     * Cleanup and store the last pose estimate in global storage.
     * Should be run in BunyipsOpMode.onStop()
     */
    public void stop() {
        // Store the last pose estimate in global storage
        RobotConfig.setLastKnownPosition(drive.getPoseEstimate());
        // Safety stop to prevent a runaway robot
        drive.stop();
    }

    public void waitForIdle() {
        drive.waitForIdle();
    }

    /**
     * For continuity, keep setSpeedUsingController for setting drive speeds.
     * Internally runs setWeightedDrivePower() and converts the controller input to a robot Pose2d.
     *
     * @param x gamepad.left_stick_x or similar
     * @param y gamepad.left_stick_y or similar
     * @param r gamepad.right_stick_x or similar
     */
    public void setSpeedUsingController(double x, double y, double r) {
        drive.setWeightedDrivePower(Controller.makeRobotPose(x, y, r));
    }

    @Override
    public void update() {
        getOpMode().addTelemetry("Localizer: X:%cm Y:%cm %deg",
                round(Inches.toCM(drive.getPoseEstimate().getX()), 1),
                round(Inches.toCM(drive.getPoseEstimate().getY()), 1),
                round(Math.toDegrees(drive.getPoseEstimate().getHeading()), 1));

        drive.update();
    }

    public MecanumRoadRunnerDrive getInstance() {
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
    @SuppressWarnings("rawtypes")
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose);
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