package system.robot.subsystems.drivetrain;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import system.robot.Robot;
import util.math.geometry.Vector2D;
import util.math.units.HALDistanceUnit;
import util.math.units.HALTimeUnit;

import static java.lang.Math.*;

public abstract class NonHolonomicDrivetrain extends Drivetrain {

    protected DriveMode driveMode = DriveMode.STANDARD;
    protected double V_WEIGHT = 1;

    protected PIDCoefficients axialCoefficients = new PIDCoefficients(1,0,0);
    protected PIDFController axialController = new PIDFController(axialCoefficients);

    protected PIDCoefficients crossTrackCoefficients = new PIDCoefficients(1,0,0);
    protected PIDFController crossTrackController = new PIDFController(crossTrackCoefficients);

    public enum DriveMode {
        STANDARD,
        DISABLED
    }

    public NonHolonomicDrivetrain(Robot robot, DriveConfig driveConfig, String... config) {
        super(robot, driveConfig, config);
    }

    protected double modifyPower(double power) {

        power *= constantSpeedMultiplier*V_WEIGHT;
        Vector2D powerVector = new Vector2D(power,0);
        velocityScaleMethod.scaleFunction.accept(powerVector);
        power = powerVector.getX()*currentSpeedMultiplier;

        return power;
    }

    protected abstract void movePowerInternal(double power);
    public final void movePower(double power) {
        movePowerInternal(modifyPower(power));
    }
    public final void moveTime(double power, long duration, HALTimeUnit timeUnit) {
        movePower(power);
        waitTime((long) HALTimeUnit.convert(duration,timeUnit,HALTimeUnit.MILLISECONDS), () -> localizer.update());
        stopAllMotors();
    }
    public final void moveTime(double power, long durationMs) {
        moveTime(power, durationMs, HALTimeUnit.MILLISECONDS);
    }
    public final void moveSimple(double distance, HALDistanceUnit distanceUnit, double power) {
        Pose2d initialPose = localizer.getPoseEstimate();
        double distanceInches = HALDistanceUnit.convert(distance, distanceUnit, HALDistanceUnit.INCHES);

        movePower(power);
        waitWhile(() -> {
            Pose2d currentPose = localizerCoordinateMode.convertTo(coordinateMode).apply(localizer.getPoseEstimate());
            System.out.println(currentPose);
            return hypot(currentPose.getX()-initialPose.getX(), currentPose.getY()-initialPose.getY()) < distanceInches;
        }, () -> localizer.update());

        stopAllMotors();
    }
    public final void moveSimple(double distance, double power) {
        moveSimple(distance, HALDistanceUnit.INCHES, power);
    }

    public final void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public final void setVelocityWeight(double velocityWeight) {
        V_WEIGHT = velocityWeight;
    }

    public final void setAxialPID(PIDCoefficients pidCoefficients) {
        axialCoefficients = pidCoefficients;
        axialController = new PIDFController(pidCoefficients);
    }

    public final void setCrossTrackPID(PIDCoefficients pidCoefficients) {
        crossTrackCoefficients = pidCoefficients;
        crossTrackController = new PIDFController(crossTrackCoefficients);
    }
}
