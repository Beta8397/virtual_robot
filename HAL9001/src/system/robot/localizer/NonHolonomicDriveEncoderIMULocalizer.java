package system.robot.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.TankKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import system.robot.Robot;
import system.robot.roadrunner_util.AxesSigns;
import system.robot.roadrunner_util.CoordinateMode;
import system.robot.subsystems.drivetrain.TankDriveSimple;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class NonHolonomicDriveEncoderIMULocalizer implements Localizer {

    private final DcMotorEx[] leftMotors, rightMotors;
    private final TankDriveSimple drivetrain;

    private final BNO055IMU imu;

    private Pose2d poseEstimate = new Pose2d(0,0,0);
    private Pose2d poseVelocity = new Pose2d(0,0,0);
    private List<Double> lastWheelPositions = new ArrayList<>();
    private double lastHeading = Double.NaN;

    public NonHolonomicDriveEncoderIMULocalizer(Robot robot, TankDriveSimple drivetrain, String imu, BNO055IMU.Parameters imuParameters, String[] leftMotors, String[] rightMotors) {

        this.drivetrain = drivetrain;

        this.imu = robot.hardwareMap.get(BNO055IMU.class, imu);
        this.imu.initialize(imuParameters);

        this.leftMotors = new DcMotorEx[leftMotors.length];
        for (int i = 0; i < leftMotors.length; i++) {
            this.leftMotors[i] = drivetrain.getMotor(leftMotors[i]);
        }

        this.rightMotors = new DcMotorEx[rightMotors.length];
        for (int i = 0; i < rightMotors.length; i++) {
            this.rightMotors[i] = drivetrain.getMotor(rightMotors[i]);
        }
    }

    public NonHolonomicDriveEncoderIMULocalizer(Robot robot, TankDriveSimple drivetrain, String imu, String[] leftMotors, String[] rightMotors) {
        this(robot, drivetrain, imu, new BNO055IMU.Parameters(), leftMotors, rightMotors);
    }

    public NonHolonomicDriveEncoderIMULocalizer(Robot robot,TankDriveSimple drivetrain, String imu, BNO055IMU.Parameters imuParameters, String leftMotor, String rightMotor) {
        this(robot, drivetrain, imu, imuParameters, new String[] {leftMotor}, new String[] {rightMotor});
    }

    public NonHolonomicDriveEncoderIMULocalizer(Robot robot, TankDriveSimple drivetrain, String imu, String leftMotor, String rightMotor) {
        this(robot, drivetrain, imu, new String[] {leftMotor}, new String[] {rightMotor});
    }

    public NonHolonomicDriveEncoderIMULocalizer(Robot robot, TankDriveSimple drivetrain, String imu, BNO055IMU.Parameters imuParameters, String topLeft, String topRight, String botLeft, String botRight) {
        this(robot, drivetrain, imu, imuParameters, new String[] {topLeft, botLeft}, new String[] {topRight, botRight});
    }

    public NonHolonomicDriveEncoderIMULocalizer(Robot robot, TankDriveSimple drivetrain, String imu, String topLeft, String topRight, String botLeft, String botRight) {
        this(robot, drivetrain, imu, new String[] {topLeft, botLeft}, new String[] {topRight, botRight});
    }

    public NonHolonomicDriveEncoderIMULocalizer remapIMUAxes(AxesOrder axesOrder, AxesSigns axesSigns) {
        //TODO BNO055IMUUtil.remapAxes(imu, axesOrder, axesSigns);
        return this;
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return CoordinateMode.ROADRUNNER.convertTo(CoordinateMode.HAL).apply(poseEstimate);
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        lastWheelPositions = new ArrayList<>();
        poseEstimate = CoordinateMode.HAL.convertTo(CoordinateMode.ROADRUNNER).apply(pose2d);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return CoordinateMode.ROADRUNNER.convertTo(CoordinateMode.HAL).apply(poseVelocity);
    }

    @Override
    public void update() {
        double leftPosition = 0;
        double leftVelocity = 0;
        for (DcMotorEx motor : leftMotors) {
            leftPosition += drivetrain.driveConfig.encoderTicksToInches(motor.getCurrentPosition());
            leftVelocity += drivetrain.driveConfig.encoderTicksToInches(motor.getVelocity());
        }
        leftPosition /= leftMotors.length;
        leftVelocity /= leftMotors.length;

        double rightPosition = 0;
        double rightVelocity = 0;
        for (DcMotorEx motor : rightMotors) {
            rightPosition += drivetrain.driveConfig.encoderTicksToInches(motor.getCurrentPosition());
            rightVelocity += drivetrain.driveConfig.encoderTicksToInches(motor.getVelocity());
        }
        rightPosition /= rightMotors.length;
        rightVelocity /= rightMotors.length;

        List<Double> wheelPositions = Arrays.asList(
                leftPosition,
                rightPosition
        );
        double heading = imu.getAngularOrientation().firstAngle;

        if(!lastWheelPositions.isEmpty()) {
            List<Double> wheelDeltas = new ArrayList<>();

            for (int i = 0; i < wheelPositions.size(); i++) {
                wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
            }
            Pose2d robotPoseDelta = TankKinematics.wheelToRobotVelocities(
                    wheelDeltas,
                    drivetrain.driveConfig.TRACK_WIDTH
            );

            double finalHeadingDelta = Angle.normDelta(heading - lastHeading);
            poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, new Pose2d(robotPoseDelta.vec(), finalHeadingDelta));
        }

        List<Double> wheelVelocities = Arrays.asList(
                leftVelocity,
                rightVelocity
        );
        poseVelocity = TankKinematics.wheelToRobotVelocities(
                wheelVelocities,
                drivetrain.driveConfig.TRACK_WIDTH
        );

        lastWheelPositions = wheelPositions;
        lastHeading = heading;
    }
}
