package system.robot.subsystems.drivetrain;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.Range;
import system.robot.roadrunner_util.CoordinateMode;
import system.robot.Robot;
import util.math.geometry.Vector2D;
import util.math.units.HALDistanceUnit;
import util.math.units.HALTimeUnit;

import static java.lang.Math.*;

public abstract class HolonomicDrivetrain extends Drivetrain {

    protected double
            VX_WEIGHT = 1,
            VY_WEIGHT = 1;
    protected double LATERAL_MULTIPLIER = 1;

    protected PIDCoefficients translationCoefficients = new PIDCoefficients(1,0,0);
    protected PIDFController translationController = new PIDFController(translationCoefficients);

    protected DriveMode driveMode = DriveMode.STANDARD;

    public enum DriveMode {
        STANDARD,
        FIELD_CENTRIC,
        DISABLED
    }

    public HolonomicDrivetrain(Robot robot, DriveConfig driveConfig, String... config) {
        super(robot, driveConfig, config);
    }

    protected Vector2D modifyPower(Vector2D power) {
        Vector2D transformedPowerVector = new Vector2D(power.getX()*VX_WEIGHT,power.getY()*VY_WEIGHT).multiply(constantSpeedMultiplier);
        velocityScaleMethod.scaleFunction.accept(transformedPowerVector);
        transformedPowerVector.multiply(currentSpeedMultiplier);

        return transformedPowerVector;
    }

    protected abstract void movePowerInternal(Vector2D power);
    public final void movePower(Vector2D power) {
        movePowerInternal(modifyPower(power));
    }
    public final void moveTime(Vector2D power, long duration, HALTimeUnit timeUnit) {
        movePower(power);
        waitTime((long) HALTimeUnit.convert(duration,timeUnit,HALTimeUnit.MILLISECONDS), () -> localizer.update());
        stopAllMotors();
    }
    public final void moveTime(Vector2D power, long durationMs) {
        moveTime(power, durationMs, HALTimeUnit.MILLISECONDS);
    }
    public final void moveSimple(Vector2D displacement, HALDistanceUnit distanceUnit, double power) {
        Pose2d initialPose = localizerCoordinateMode.convertTo(coordinateMode).apply(localizer.getPoseEstimate());

        Vector2D transformedDisplacement = new Vector2D(
                HALDistanceUnit.convert(displacement.getX(), distanceUnit, HALDistanceUnit.INCHES),
                HALDistanceUnit.convert(displacement.getY(), distanceUnit, HALDistanceUnit.INCHES)
        );

        if(coordinateMode == CoordinateMode.ROADRUNNER) {
            transformedDisplacement.rotate(PI/2);
        }

        Vector2D velocity = transformedDisplacement.clone().normalize().multiply(Range.clip(power,-1,1)).rotate(-PI/2);

        if(driveMode == DriveMode.STANDARD) {
            transformedDisplacement.rotate(localizer.getPoseEstimate().getHeading());
        }

        movePower(velocity);
        waitWhile(() -> {
            Pose2d currentPose = localizerCoordinateMode.convertTo(coordinateMode).apply(localizer.getPoseEstimate());
            System.out.println(currentPose);
            return abs(currentPose.getX()-initialPose.getX()) < abs(transformedDisplacement.getX()) || abs(currentPose.getY()-initialPose.getY()) < abs(transformedDisplacement.getY());
        }, () -> localizer.update());

        stopAllMotors();
    }
    public final void moveSimple(Vector2D displacement, double power) {
        moveSimple(displacement, HALDistanceUnit.INCHES, power);
    }

    public final void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public final void setTranslationalPID(PIDCoefficients pidCoefficients) {
        translationCoefficients = pidCoefficients;
        translationController = new PIDFController(translationCoefficients);
    }

    public final void setVelocityXWeight(double velocityXWeight) {
        VX_WEIGHT = velocityXWeight;
    }

    public final void setVelocityYWeight(double velocityYWeight) {
        VY_WEIGHT = velocityYWeight;
    }

    public final void setLateralMultiplier(double lateralMultiplier) {
        LATERAL_MULTIPLIER = lateralMultiplier;
    }

    public double getVelocityXWeight() {
        return VX_WEIGHT;
    }

    public double getVelocityYWeight() {
        return VY_WEIGHT;
    }

    public double getLateralMultiplier() {
        return LATERAL_MULTIPLIER;
    }
}
