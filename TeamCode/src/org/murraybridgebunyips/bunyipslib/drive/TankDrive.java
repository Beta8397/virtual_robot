package org.murraybridgebunyips.bunyipslib.drive;

import static org.murraybridgebunyips.bunyipslib.Text.round;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Centimeters;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Storage;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.DriveConstants;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.TankCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.TankRoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceRunner;

import java.util.List;

/**
 * This is the standard TankDrive class for modern BunyipsLib robots.
 * This is a component for the RoadRunner Tank Drive, integrating RoadRunner and BunyipsLib to be used
 * as a BunyipsSubsystem. As such, this allows for integrated trajectory and pose management.
 *
 * @author Lucas Bubner, 2023
 */
public class TankDrive extends BunyipsSubsystem implements RoadRunnerDrive {
    private final TankRoadRunnerDrive instance;

    /**
     * Create a new TankDrive instance.
     *
     * @param constants    The drive constants
     * @param coefficients The tank coefficients
     * @param imu          The IMU
     * @param frontLeft    The front left motor
     * @param frontRight   The front right motor
     * @param backLeft     The back left motor
     * @param backRight    The back right motor
     */
    public TankDrive(DriveConstants constants, TankCoefficients coefficients, IMU imu, DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        assertParamsNotNull(constants, coefficients, imu, frontLeft, frontRight, backLeft, backRight);
        instance = new TankRoadRunnerDrive(opMode.telemetry, constants, coefficients, opMode.hardwareMap.voltageSensor, imu, frontLeft, frontRight, backLeft, backRight);
    }

    @Override
    public TrajectorySequenceRunner getTrajectorySequenceRunner() {
        return instance.getTrajectorySequenceRunner();
    }

    @Override
    public void stop() {
        instance.stop();
    }

    @Override
    public void waitForIdle() {
        instance.waitForIdle();
    }

    @Override
    public DriveConstants getConstants() {
        return instance.getConstants();
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return instance.trajectoryBuilder(startPose);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return instance.trajectoryBuilder(startPose, reversed);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return instance.trajectoryBuilder(startPose, startHeading);
    }

    @Override
    @SuppressWarnings("rawtypes")
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return instance.trajectorySequenceBuilder(startPose);
    }

    @Override
    public void turnAsync(double angle) {
        instance.turnAsync(angle);
    }

    @Override
    public void turn(double angle) {
        instance.turn(angle);
    }

    @Override
    public void followTrajectoryAsync(Trajectory trajectory) {
        instance.followTrajectoryAsync(trajectory);
    }

    @Override
    public void followTrajectory(Trajectory trajectory) {
        instance.followTrajectory(trajectory);
    }

    @Override
    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        instance.followTrajectorySequenceAsync(trajectorySequence);
    }

    @Override
    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        instance.followTrajectorySequence(trajectorySequence);
    }

    @Override
    public Pose2d getLastError() {
        return instance.getLastError();
    }

    @Override
    protected void periodic() {
        opMode.telemetry.add("Localizer: X:%cm Y:%cm %deg",
                round(Centimeters.convertFrom(instance.getPoseEstimate().getX(), Inches), 1),
                round(Centimeters.convertFrom(instance.getPoseEstimate().getY(), Inches), 1),
                round(Math.toDegrees(instance.getPoseEstimate().getHeading()), 1)).color("gray");

        instance.update();
        Storage.memory().lastKnownPosition = instance.getPoseEstimate();
    }

    @Override
    public boolean isBusy() {
        return instance.isBusy();
    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {
        instance.setMode(runMode);
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        instance.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        instance.setPIDFCoefficients(runMode, coefficients);
    }

    @Override
    public void setWeightedDrivePower(Pose2d drivePower) {
        instance.setWeightedDrivePower(drivePower);
    }

    @Override
    public List<Double> getWheelPositions() {
        return instance.getWheelPositions();
    }

    @Override
    public List<Double> getWheelVelocities() {
        return instance.getWheelVelocities();
    }

    @Override
    public double getRawExternalHeading() {
        return instance.getRawExternalHeading();
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return instance.getExternalHeadingVelocity();
    }

    @Override
    public Localizer getLocalizer() {
        return instance.getLocalizer();
    }

    @Override
    public void setLocalizer(Localizer localizer) {
        instance.setLocalizer(localizer);
    }

    @Override
    public double getExternalHeading() {
        return instance.getExternalHeading();
    }

    @Override
    public void setExternalHeading(double value) {
        instance.setExternalHeading(value);
    }

    @Override
    public Pose2d getPoseEstimate() {
        return instance.getPoseEstimate();
    }

    @Override
    public void setPoseEstimate(Pose2d value) {
        instance.setPoseEstimate(value);
    }

    @Override
    public Pose2d getPoseVelocity() {
        return instance.getPoseVelocity();
    }

    @Override
    public void updatePoseEstimate() {
        instance.updatePoseEstimate();
    }

    @Override
    public void setDriveSignal(DriveSignal driveSignal) {
        instance.setDriveSignal(driveSignal);
    }

    @Override
    public void setDrivePower(Pose2d drivePower) {
        instance.setDrivePower(drivePower);
    }

    @Override
    public void setWeightedDrivePowerFieldCentric(Pose2d pose) {
        instance.setWeightedDrivePowerFieldCentric(pose);
    }

    /**
     * Set the speed of the drive using the controller input.
     *
     * @param x The x value of the controller input
     * @param y The y value of the controller input
     * @param r The r value of the controller input
     * @return The TankDrive instance
     */
    public TankDrive setSpeedUsingController(double x, double y, double r) {
        setWeightedDrivePower(Controls.makeRobotPose(x, y, r));
        return this;
    }

    /**
     * Set the speed of the drive using the controller input, field centric.
     *
     * @param x The x value of the controller input
     * @param y The y value of the controller input
     * @param r The r value of the controller input
     * @return The TankDrive instance
     */
    public TankDrive setSpeedUsingControllerFieldCentric(double x, double y, double r) {
        setWeightedDrivePowerFieldCentric(Controls.makeRobotPose(x, y, r));
        return this;
    }

    @Override
    public void cancelTrajectory() {
        instance.cancelTrajectory();
    }
}
