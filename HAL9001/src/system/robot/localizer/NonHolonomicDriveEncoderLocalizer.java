package system.robot.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.TankKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import system.robot.roadrunner_util.CoordinateMode;
import system.robot.subsystems.drivetrain.TankDriveSimple;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class NonHolonomicDriveEncoderLocalizer implements Localizer {

    private final DcMotorEx[] leftMotors, rightMotors;
    private final TankDriveSimple drivetrain;

    private Pose2d poseEstimate = new Pose2d(0,0,0);
    private Pose2d poseVelocity = new Pose2d(0,0,0);
    private List<Double> lastWheelPositions = new ArrayList<>();
    private double lastHeading = Double.NaN;


    public NonHolonomicDriveEncoderLocalizer(TankDriveSimple drivetrain, String[] leftMotors, String[] rightMotors) {

        this.drivetrain = drivetrain;

        this.leftMotors = new DcMotorEx[leftMotors.length];
        for (int i = 0; i < leftMotors.length; i++) {
            this.leftMotors[i] = drivetrain.getMotor(leftMotors[i]);
        }

        this.rightMotors = new DcMotorEx[rightMotors.length];
        for (int i = 0; i < rightMotors.length; i++) {
            this.rightMotors[i] = drivetrain.getMotor(rightMotors[i]);
        }
    }

    public NonHolonomicDriveEncoderLocalizer(TankDriveSimple drivetrain, String leftMotor, String rightMotor) {
        this(drivetrain, new String[] {leftMotor}, new String[] {rightMotor});
    }

    public NonHolonomicDriveEncoderLocalizer(TankDriveSimple drivetrain, String topLeft, String topRight, String botLeft, String botRight) {
        this(drivetrain, new String[] {topLeft, botLeft}, new String[] {topRight, botRight});
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
        double heading = (wheelPositions.get(1)-wheelPositions.get(0))/drivetrain.driveConfig.TRACK_WIDTH;
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
                drivetrain.driveConfig.encoderTicksToInches(leftVelocity),
                drivetrain.driveConfig.encoderTicksToInches(rightVelocity)
        );
        poseVelocity = TankKinematics.wheelToRobotVelocities(
                wheelVelocities,
                drivetrain.driveConfig.TRACK_WIDTH
        );

        lastWheelPositions = wheelPositions;
        lastHeading = heading;
    }
}
