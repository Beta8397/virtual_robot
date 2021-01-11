package system.robot.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
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
import system.robot.subsystems.drivetrain.HolonomicDrivetrain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class HolonomicDriveEncoderIMULocalizer implements Localizer {

    private final DcMotorEx topLeft, botLeft, topRight, botRight;
    private final BNO055IMU imu;
    private final HolonomicDrivetrain drivetrain;

    private Pose2d poseEstimate = new Pose2d(0,0,0);
    private Pose2d poseVelocity = new Pose2d(0,0,0);
    private List<Double> lastWheelPositions = new ArrayList<>();
    private double lastHeading = Double.NaN;


    public HolonomicDriveEncoderIMULocalizer(Robot robot, HolonomicDrivetrain drivetrain, String imu, BNO055IMU.Parameters imuParameters, String topLeft, String topRight, String botLeft, String botRight) {

        this.drivetrain = drivetrain;

        this.imu = robot.hardwareMap.get(BNO055IMU.class, imu);
        this.topLeft = drivetrain.getMotor(topLeft);
        this.topRight = drivetrain.getMotor(topRight);
        this.botLeft = drivetrain.getMotor(botLeft);
        this.botRight = drivetrain.getMotor(botRight);

        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        this.imu.initialize(imuParameters);

    }

    public HolonomicDriveEncoderIMULocalizer(Robot robot, HolonomicDrivetrain drivetrain, String imu, String topLeft, String topRight, String botLeft, String botRight) {
        this(robot, drivetrain, imu, new BNO055IMU.Parameters(), topLeft, topRight, botLeft, botRight);
    }

    public HolonomicDriveEncoderIMULocalizer remapIMUAxes(AxesOrder axesOrder, AxesSigns axesSigns) {
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

        List<Double> wheelPositions = Arrays.asList(
                drivetrain.driveConfig.encoderTicksToInches(topLeft.getCurrentPosition()),
                drivetrain.driveConfig.encoderTicksToInches(botLeft.getCurrentPosition()),
                drivetrain.driveConfig.encoderTicksToInches(botRight.getCurrentPosition()),
                drivetrain.driveConfig.encoderTicksToInches(topRight.getCurrentPosition())
        );
        double heading = imu.getAngularOrientation().firstAngle;
        if(!lastWheelPositions.isEmpty()) {
            List<Double> wheelDeltas = new ArrayList<>();

            for (int i = 0; i < wheelPositions.size(); i++) {
                wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
            }
            Pose2d robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(
                    wheelDeltas,
                    drivetrain.driveConfig.TRACK_WIDTH,
                    drivetrain.driveConfig.WHEEL_BASE,
                    drivetrain.getLateralMultiplier()
            );

            double finalHeadingDelta = Angle.normDelta(heading - lastHeading);
            poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, new Pose2d(robotPoseDelta.vec(), finalHeadingDelta));
        }

        List<Double> wheelVelocities = Arrays.asList(
                drivetrain.driveConfig.encoderTicksToInches(topLeft.getVelocity()),
                drivetrain.driveConfig.encoderTicksToInches(botLeft.getVelocity()),
                drivetrain.driveConfig.encoderTicksToInches(botRight.getVelocity()),
                drivetrain.driveConfig.encoderTicksToInches(topRight.getVelocity())
        );
        poseVelocity = MecanumKinematics.wheelToRobotVelocities(
                wheelVelocities,
                drivetrain.driveConfig.TRACK_WIDTH,
                drivetrain.driveConfig.WHEEL_BASE,
                drivetrain.getLateralMultiplier()
        );

        lastWheelPositions = wheelPositions;
        lastHeading = heading;
    }
}
