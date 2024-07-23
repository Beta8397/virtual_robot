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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Storage;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.DriveConstants;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumRoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceRunner;

import java.util.List;

/**
 * This is the standard MecanumDrive class for modern BunyipsLib robots.
 * This is a component for the RoadRunner Mecanum Drive, integrating RoadRunner and BunyipsLib to be used
 * as a BunyipsSubsystem. As such, this allows for integrated trajectory and pose management,
 * as well as the ability to use Field Centric Drive as a native method.
 *
 * @author Lucas Bubner, 2023
 */
public class MecanumDrive extends BunyipsSubsystem implements RoadRunnerDrive {
    private final MecanumRoadRunnerDrive drive;
    private final IMU imu;

    /**
     * Constructor for the MecanumDrive class.
     *
     * @param constants           The drive constants for the robot.
     * @param mecanumCoefficients The coefficients for the mecanum drive.
     * @param voltageSensor       The voltage sensor for the robot from hardwareMap.
     * @param imu                 The IMU for the robot.
     * @param fl                  The front left motor.
     * @param fr                  The front right motor.
     * @param bl                  The back left motor.
     * @param br                  The back right motor.
     */
    public MecanumDrive(DriveConstants constants, MecanumCoefficients mecanumCoefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, IMU imu, DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br) {
        assertParamsNotNull(constants, mecanumCoefficients, voltageSensor, imu, fl, fr, bl, br);
        drive = new MecanumRoadRunnerDrive(opMode.telemetry, constants, mecanumCoefficients, voltageSensor, imu, fl, fr, bl, br);
        // If we have a last known position, set the pose estimate to it
        if (Storage.memory().lastKnownPosition != null) {
            drive.setPoseEstimate(Storage.memory().lastKnownPosition);
        }
        this.imu = imu;
    }

    /**
     * Reset the IMU's yaw to 0.
     */
    public void resetYaw() {
        imu.resetYaw();
        setExternalHeading(0);
    }

    @Override
    public TrajectorySequenceRunner getTrajectorySequenceRunner() {
        return drive.getTrajectorySequenceRunner();
    }

    /**
     * Cleanup and store the last pose estimate in global storage, while stopping the motors.
     */
    @Override
    public void stop() {
        // Store the last pose estimate in global storage
        Storage.memory().lastKnownPosition = drive.getPoseEstimate();
        // Safety stop to prevent a runaway robot
        drive.stop();
    }

    public void waitForIdle() {
        drive.waitForIdle();
    }

    @Override
    public DriveConstants getConstants() {
        return drive.getConstants();
    }

    /**
     * For continuity, keep setSpeedUsingController for setting drive speeds.
     * Internally runs setWeightedDrivePower() and converts the controller input to a robot Pose2d.
     *
     * @param x gamepad.left_stick_x or similar
     * @param y gamepad.left_stick_y or similar
     * @param r gamepad.right_stick_x or similar
     * @return this
     */
    public MecanumDrive setSpeedUsingController(double x, double y, double r) {
        drive.setWeightedDrivePower(Controls.makeRobotPose(x, y, r));
        return this;
    }

    @Override
    protected void periodic() {
        opMode.telemetry.add("Localizer: X:%cm Y:%cm %deg",
                round(Centimeters.convertFrom(drive.getPoseEstimate().getX(), Inches), 1),
                round(Centimeters.convertFrom(drive.getPoseEstimate().getY(), Inches), 1),
                round(Math.toDegrees(drive.getPoseEstimate().getHeading()), 1)).color("gray");

        drive.update();
        Storage.memory().lastKnownPosition = drive.getPoseEstimate();
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

    /**
     * Set the raw motor powers for the drive.
     *
     * @param v  The power for the front left motor.
     * @param v1 The power for the front right motor.
     * @param v2 The power for the back left motor.
     * @param v3 The power for the back right motor.
     */
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
    public void setWeightedDrivePowerFieldCentric(Pose2d pose) {
        drive.setWeightedDrivePowerFieldCentric(pose);
    }

    /**
     * Set the speed of the robot using field centric drive using a gamepad.
     *
     * @param x The x speed.
     * @param y The y speed.
     * @param r The rotation speed.
     * @see #setWeightedDrivePowerFieldCentric(Pose2d)
     */
    public void setSpeedUsingControllerFieldCentric(double x, double y, double r) {
        setWeightedDrivePowerFieldCentric(Controls.makeRobotPose(x, y, r));
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

    public void cancelTrajectory() {
        drive.cancelTrajectory();
    }
}