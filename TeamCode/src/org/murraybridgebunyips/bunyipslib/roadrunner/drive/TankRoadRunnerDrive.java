package org.murraybridgebunyips.bunyipslib.roadrunner.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Controller;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * RoadRunner Tank drive hardware implementation for REV hardware.
 * Reworked to use a builder parameters for multiple robot configurations.
 */
public class TankRoadRunnerDrive extends com.acmerobotics.roadrunner.drive.TankDrive implements RoadRunnerDrive {
    private final DriveConstants constants;
    private final TankCoefficients coefficients;

    private final TrajectorySequenceRunner trajectorySequenceRunner;

    private final TrajectoryVelocityConstraint VEL_CONSTRAINT;
    private final TrajectoryAccelerationConstraint accelConstraint;

    private final TrajectoryFollower follower;

    private final List<DcMotorEx> motors;
    private final List<DcMotorEx> leftMotors;
    private final List<DcMotorEx> rightMotors;
    private final IMU imu;

    private final VoltageSensor batteryVoltageSensor;

    public TankRoadRunnerDrive(@Nullable BunyipsOpMode opMode, DriveConstants constants, TankCoefficients coefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, IMU imu, DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br) {
        super(constants.kV, constants.kA, constants.kStatic, constants.TRACK_WIDTH);

        follower = new TankPIDVAFollower(coefficients.AXIAL_PID, coefficients.CROSS_TRACK_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        VEL_CONSTRAINT = getVelocityConstraint(constants.MAX_VEL, constants.MAX_ANG_VEL, constants.TRACK_WIDTH);
        accelConstraint = getAccelerationConstraint(constants.MAX_ACCEL);

        batteryVoltageSensor = voltageSensor.iterator().next();

        this.constants = constants;
        this.coefficients = coefficients;

        assert fl != null && fr != null && bl != null && br != null && imu != null;

        // Assumes IMU is initialised from RobotConfig
        this.imu = imu;

        motors = Arrays.asList(fl, bl, br, fr);
        leftMotors = Arrays.asList(fl, bl);
        rightMotors = Arrays.asList(br, fr);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (constants.RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (constants.RUN_USING_ENCODER && constants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, constants.MOTOR_VELO_PID);
        }

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                opMode, constants.RUN_USING_ENCODER, follower, coefficients.HEADING_PID, batteryVoltageSensor,
                new ArrayList<>(), new ArrayList<>(), new ArrayList<>(), new ArrayList<>()
        );
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new TankVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    @Override
    public TrajectorySequenceRunner getTrajectorySequenceRunner() {
        return trajectorySequenceRunner;
    }

    @Override
    public void stop() {
        setMotorPowers(0, 0);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, accelConstraint);
    }

    @SuppressWarnings("rawtypes")
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, accelConstraint,
                constants.MAX_ANG_VEL, constants.MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    @Override
    public DriveConstants getConstants() {
        return constants;
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = coefficients.VX_WEIGHT * Math.abs(drivePower.getX())
                    + coefficients.OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    coefficients.VX_WEIGHT * drivePower.getX(),
                    0,
                    coefficients.OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        } else {
            // Ensure the y axis is zeroed out.
            vel = new Pose2d(drivePower.getX(), 0, drivePower.getHeading());
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        double leftSum = 0, rightSum = 0;
        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += constants.encoderTicksToInches(leftMotor.getCurrentPosition());
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += constants.encoderTicksToInches(rightMotor.getCurrentPosition());
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }

    public List<Double> getWheelVelocities() {
        double leftSum = 0, rightSum = 0;
        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += constants.encoderTicksToInches(leftMotor.getVelocity());
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += constants.encoderTicksToInches(rightMotor.getVelocity());
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }

    @Override
    public void setMotorPowers(double v, double v1) {
        for (DcMotorEx leftMotor : leftMotors) {
            leftMotor.setPower(v);
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightMotor.setPower(v1);
        }
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    @Override
    public void setWeightedDrivePowerFieldCentric(Pose2d pose) {
        double heading = getExternalHeading();
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);
        setWeightedDrivePower(new Pose2d(
                pose.getY() * sin + pose.getX() * cos,
                pose.getY() * cos - pose.getX() * sin,
                pose.getHeading()
        ));
    }

    public void setSpeedUsingControllerFieldCentric(double x, double y, double r) {
        setWeightedDrivePowerFieldCentric(Controller.makeRobotPose(x, y, r));
    }

    @Override
    public void cancelTrajectory() {
        trajectorySequenceRunner.cancelTrajectory();
    }
}
