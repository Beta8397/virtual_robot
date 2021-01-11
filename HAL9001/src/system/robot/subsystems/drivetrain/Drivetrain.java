package system.robot.subsystems.drivetrain;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import system.robot.roadrunner_util.CoordinateMode;
import system.robot.Robot;
import system.robot.SubSystem;
import util.math.geometry.Vector2D;
import util.math.units.HALAngleUnit;
import util.math.units.HALTimeUnit;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

import static java.lang.Math.PI;
import static java.lang.Math.signum;
import static java.lang.Math.abs;
import static java.lang.Math.min;

public abstract class Drivetrain extends SubSystem {

    protected Map<String, DcMotorEx> motors = new HashMap<>();
    protected Localizer localizer;
    protected CoordinateMode coordinateMode, localizerCoordinateMode;

    public final DriveConfig driveConfig;

    protected PIDCoefficients
            headingCoefficients = new PIDCoefficients(2,0,0),
            turnCoefficients = new PIDCoefficients(1,0,0);
    protected PIDFController
            headingController = new PIDFController(headingCoefficients),
            turnController = new PIDFController(turnCoefficients);

    protected SpeedScaleMethod
            velocityScaleMethod = SpeedScaleMethod.NONE,
            turnSpeedScaleMethod = SpeedScaleMethod.NONE;

    protected double
            constantSpeedMultiplier = 1,
            constantTurnSpeedMultiplier = 1,
            currentSpeedMultiplier = 1,
            currentTurnSpeedMultiplier = 1,
            velocityCap = 1,
            turnSpeedCap = 1;

    protected double OMEGA_WEIGHT = 1;

    protected double headingAngleToleranceRadians = 1e-2;

    public enum SpeedScaleMethod {
        NONE((Vector2D velocity) -> {}),
        SQUARE((Vector2D velocity) -> {
            if(!velocity.isZeroVector()) {
                double magnitude = velocity.magnitude();
                velocity.normalize().multiply(magnitude * magnitude);
            }
        }),
        CUBIC((Vector2D velocity) -> {
            if(!velocity.isZeroVector()) {
                double magnitude = velocity.magnitude();
                velocity.normalize().multiply(magnitude * magnitude * magnitude);
            }
        });

        public final Consumer<Vector2D> scaleFunction;
        SpeedScaleMethod(Consumer<Vector2D> scaleFunction) {
            this.scaleFunction = scaleFunction;
        }
    }

    public Drivetrain(Robot robot, DriveConfig driveConfig, String... config) {
        super(robot);

        this.driveConfig = driveConfig;

        for (String motorName : config) {
            motors.put(motorName, robot.hardwareMap.get(DcMotorEx.class, motorName));
        }
        resetMotorEncoders();
        setAllMotorZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        turnController.setInputBounds(-PI, PI);
        headingController.setInputBounds(-PI, PI);

        headingController.setTargetPosition(0);

        coordinateMode = CoordinateMode.HAL;
        localizerCoordinateMode = CoordinateMode.HAL;
    }

    public final String[] getMotorConfig() {
        return motors.keySet().toArray(new String[0]);
    }

    public final DcMotorEx[] getMotors() {
        return motors.values().toArray(new DcMotorEx[0]);
    }

    public final DcMotorEx getMotor(String motorName) {
        return motors.get(motorName);
    }

    public final void resetMotorEncoders() {
        for(DcMotorEx motor : motors.values()) {
            DcMotor.RunMode runMode = motor.getMode();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(runMode);
        }
    }

    public final void stopAllMotors() {
        for(DcMotorEx motor : motors.values()) {
            motor.setPower(0);
        }
    }

    public final void setAllMotorZeroPowerBehaviors(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for(DcMotorEx motor : motors.values()) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public final void setAllMotorModes(DcMotor.RunMode runMode) {
        for(DcMotorEx motor : motors.values()) {
            motor.setMode(runMode);
        }
    }

    public final void setMotorPower(String motorName, double motorPower) {
        getMotor(motorName).setPower(motorPower);
    }

    public final void setMotorMode(String motorName, DcMotor.RunMode mode) {
        getMotor(motorName).setMode(mode);
    }

    public final void setMotorZeroPowerBehavior(String motorName, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        getMotor(motorName).setZeroPowerBehavior(zeroPowerBehavior);
    }

    public final void reverseMotor(String motorName) {
        DcMotorEx motor = getMotor(motorName);
        motor.setDirection(motor.getDirection() == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public final void setMotorDirection(String motorName, DcMotorSimple.Direction direction) {
       getMotor(motorName).setDirection(direction);
    }

    public final int getMotorEncoderPosition(String motorName) {
        return getMotor(motorName).getCurrentPosition();
    }

    public final double getMotorVelocity(String motorName, HALAngleUnit angleUnit) {
        return getMotor(motorName).getVelocity(angleUnit == HALAngleUnit.RADIANS ? AngleUnit.RADIANS : AngleUnit.DEGREES);
    }

    public final double getMotorVelocity(String motorName) {
        return getMotorVelocity(motorName, HALAngleUnit.RADIANS);
    }

    protected double modifyTurnPower(double turnPower) {
        turnPower *= constantTurnSpeedMultiplier*OMEGA_WEIGHT;
        turnPower = signum(turnPower)*min(abs(turnPower), turnSpeedCap);
        Vector2D turnVector = new Vector2D(turnPower, 0);
        turnSpeedScaleMethod.scaleFunction.accept(turnVector);

        turnPower = turnVector.getX()*currentTurnSpeedMultiplier;
        return turnPower;
    }

    public abstract void turnPowerInternal(double power);
    public final void turnPower(double power) {
        power = modifyTurnPower(power);
        turnPowerInternal(power);
    }

    public final void turnTime(double power, double duration, HALTimeUnit timeUnit) {
        turnPower(power);
        waitTime((long) HALTimeUnit.convert(duration, timeUnit, HALTimeUnit.MILLISECONDS), () -> localizer.update());
        stopAllMotors();
    }

    public final void turnTime(double power, long durationMs) {
        turnTime(power, durationMs, HALTimeUnit.MILLISECONDS);
    }

    public final void turnSimple(double power, double amount) {
        Pose2d initialPose = localizer.getPoseEstimate();

        if(power != 0) {
            turnPower(amount < 0 ? -power : power);
            waitWhile(() -> abs(localizer.getPoseEstimate().getHeading() - initialPose.getHeading()) < abs(amount), () -> {
                localizer.update();
                System.out.println(localizer.getPoseEstimate());
            });
            stopAllMotors();
        }
    }

    public final void turnSimple(double power, double angle, HALAngleUnit angleUnit) {
        turnSimple(power, angleUnit.convertTo(HALAngleUnit.RADIANS).apply(angle));
    }

    public final void turnPID(double targetAngle, HALAngleUnit angleUnit, double angleTolerance, HALAngleUnit toleranceUnit) {
        turnController.setTargetPosition(angleUnit.convertTo(HALAngleUnit.RADIANS).apply(targetAngle));
        turnController.update(localizer.getPoseEstimate().getHeading());

        angleTolerance = toleranceUnit.convertTo(HALAngleUnit.RADIANS).apply(abs(angleTolerance));

        while(robot.opModeIsActive() && abs(turnController.getLastError()) > angleTolerance) {
            localizer.update();
            double correction = turnController.update(localizer.getPoseEstimate().getHeading());
            if(correction == 0) break;
            turnPowerInternal(correction);
        }
        stopAllMotors();
    }

    public final void turnPID(double targetAngle, HALAngleUnit angleUnit, double angleToleranceRadians) {
        turnPID(targetAngle, angleUnit, angleToleranceRadians, HALAngleUnit.RADIANS);
    }

    public final void turnPID(double targetAngleRadians, double angleToleranceRadians) {
        turnPID(targetAngleRadians, HALAngleUnit.RADIANS, angleToleranceRadians, HALAngleUnit.RADIANS);
    }

    public final void turnPID(double targetAngle, HALAngleUnit angleUnit) {
        turnPID(targetAngle, angleUnit, 1e-2, HALAngleUnit.RADIANS);
    }

    public final void turnPID(double targetAngleRadians) {
        turnPID(targetAngleRadians, HALAngleUnit.RADIANS);
    }

    public final void setTurnPID(PIDCoefficients pidCoefficients) {
        turnCoefficients = pidCoefficients;
        turnController = new PIDFController(turnCoefficients);
    }

    public final void setHeadingPID(PIDCoefficients pidCoefficients) {
        headingCoefficients = pidCoefficients;
        headingController = new PIDFController(headingCoefficients);
    }

    public final void setHeadingPIDTolerance(double tolerance, HALAngleUnit toleranceUnit) {
        headingAngleToleranceRadians = toleranceUnit.convertTo(HALAngleUnit.RADIANS).apply(tolerance);
    }

    public final void setHeadingPIDTolerance(double toleranceRadians) {
        setHeadingPIDTolerance(toleranceRadians, HALAngleUnit.RADIANS);
    }

    public final void setVelocityScaleMethod(SpeedScaleMethod velocityScaleMethod) {
        this.velocityScaleMethod = velocityScaleMethod;
    }

    public final void setTurnSpeedScaleMethod(SpeedScaleMethod turnSpeedScaleMethod) {
        this.turnSpeedScaleMethod = turnSpeedScaleMethod;
    }

    public final void setVelocityCap(double velocityCap) {
        if(velocityCap > 0) {
            this.velocityCap = Range.clip(velocityCap, 0, 1);
        }
    }

    public final void setTurnSpeedCap(double turnSpeedCap) {
        if(turnSpeedCap > 0) {
            this.turnSpeedCap = Range.clip(turnSpeedCap, 0, 1);
        }
    }

    public final void setVelocityMultiplier(double velocityMultiplier) {
        constantSpeedMultiplier = abs(velocityMultiplier);
    }

    public final void setTurnSpeedMultiplier(double turnSpeedMultiplier) {
        constantTurnSpeedMultiplier = abs(turnSpeedMultiplier);
    }

    public final void setAngularVelocityWeight(double angularVelocityWeight) {
        OMEGA_WEIGHT = angularVelocityWeight;
    }

    public void setLocalizer(Localizer localizer, CoordinateMode localizerCoordinateMode) {
        this.localizer = localizer;
        this.localizerCoordinateMode = localizerCoordinateMode;
    }

    public void setLocalizer(Localizer localizer) {
        setLocalizer(localizer, CoordinateMode.HAL);
    }

    public final void setCoordinateMode(CoordinateMode coordinateMode) {
        this.coordinateMode = coordinateMode;
    }

    public final Pose2d getPoseEstimate() {
        return localizer.getPoseEstimate();
    }
}