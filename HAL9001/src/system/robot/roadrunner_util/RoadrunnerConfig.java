package system.robot.roadrunner_util;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import system.robot.subsystems.drivetrain.DriveConfig;
import util.math.units.*;

public class RoadrunnerConfig extends DriveConfig {

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public boolean USE_DRIVE_ENCODERS;
    public PIDFCoefficients MOTOR_VELO_PID;

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public double kV;
    public double kA;
    public double kStatic;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    public double MAX_VEL;
    public double MAX_ACCEL;
    public double MAX_ANG_VEL;
    public double MAX_ANG_ACCEL;

    public double FOLLOWER_X_TOLERANCE = 0.5;
    public double FOLLOWER_Y_TOLERANCE = 0.5;
    public double FOLLOWER_HEADING_TOLERANCE = Math.toRadians(5.0);
    public double FOLLOWER_TIMEOUT = 0.5;

    public RoadrunnerConfig(double wheelRadius, HALDistanceUnit wheelRadiusUnit, double gearRatio, double trackWidth, HALDistanceUnit trackWidthUnit, double wheelBase, HALDistanceUnit wheelBaseUnit, double motorTicksPerRevolution, double motorMaxRPM) {
        super(wheelRadius, wheelRadiusUnit, gearRatio, trackWidth, trackWidthUnit, wheelBase, wheelBaseUnit, motorTicksPerRevolution, motorMaxRPM);
        USE_DRIVE_ENCODERS = true;
        MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

        kV = 1.0 / rpmToVelocity(MAX_RPM);
        kA = 0;
        kStatic = 0;

        MAX_VEL = 30;
        MAX_ACCEL = 30;
        MAX_ANG_VEL = Math.toRadians(180);
        MAX_ANG_ACCEL = Math.toRadians(180);
    }

    public RoadrunnerConfig(double wheelRadiusInches, double gearRatio, double trackWidthInches, double wheelBaseInches, double motorTicksPerRevolution, double motorMaxRPM) {
        this(wheelRadiusInches, HALDistanceUnit.INCHES, gearRatio, trackWidthInches, HALDistanceUnit.INCHES, wheelBaseInches, HALDistanceUnit.INCHES, motorTicksPerRevolution, motorMaxRPM);
    }

    public RoadrunnerConfig(double wheelRadiusInches, double gearRatio, double trackWidthInches, double motorTicksPerRevolution, double motorMaxRPM) {
        this(wheelRadiusInches, gearRatio, trackWidthInches, trackWidthInches, motorTicksPerRevolution, motorMaxRPM);
    }

    public RoadrunnerConfig useDriveEncoders(boolean useDriveEncoders) {
        USE_DRIVE_ENCODERS = useDriveEncoders;
        return this;
    }

    public RoadrunnerConfig setMotorVelocityPID(PIDFCoefficients motorVelocityPIDCoefficients) {
        MOTOR_VELO_PID = motorVelocityPIDCoefficients;
        return this;
    }

    public RoadrunnerConfig setKV(double kV) {
        this.kV = kV;
        return this;
    }

    public RoadrunnerConfig setKA(double kA) {
        this.kA = kA;
        return this;
    }

    public RoadrunnerConfig setKStatic(double kStatic) {
        this.kStatic = kStatic;
        return this;
    }

    public RoadrunnerConfig setMaxVelocity(double maxVelocity, HALVelocityUnit velocityUnit) {
        MAX_VEL = HALVelocityUnit.convert(maxVelocity, velocityUnit, HALVelocityUnit.INCHES_PER_SECOND);
        return this;
    }

    public RoadrunnerConfig setMaxAcceleration(double maxAcceleration, HALAccelerationUnit accelerationUnit) {
        MAX_ACCEL = HALAccelerationUnit.convert(maxAcceleration,accelerationUnit,HALAccelerationUnit.INCHES_PER_SECOND_SQUARED);
        return this;
    }

    public RoadrunnerConfig setMaxAngularVelocity(double maxAngularVelocity, HALAngularVelocityUnit angularVelocityUnit) {
        MAX_ANG_VEL = HALAngularVelocityUnit.convert(maxAngularVelocity,angularVelocityUnit,HALAngularVelocityUnit.RADIANS_PER_SECOND);
        return this;
    }

    public RoadrunnerConfig setMaxAngularAcceleration(double maxAngularAcceleration, HALAngularAccelerationUnit angularAccelerationUnit) {
        MAX_ANG_ACCEL = HALAngularAccelerationUnit.convert(maxAngularAcceleration,angularAccelerationUnit,HALAngularAccelerationUnit.RADIANS_PER_SECOND_SQUARED);
        return this;
    }

    public RoadrunnerConfig setFollowerXTolerance(double xTolerance, HALDistanceUnit toleranceUnit) {
        FOLLOWER_X_TOLERANCE = HALDistanceUnit.convert(xTolerance, toleranceUnit, HALDistanceUnit.INCHES);
        return this;
    }

    public RoadrunnerConfig setFollowerXTolerance(double xToleranceInches) {
        setFollowerXTolerance(xToleranceInches, HALDistanceUnit.INCHES);
        return this;
    }

    public RoadrunnerConfig setFollowerYTolerance(double yTolerance, HALDistanceUnit toleranceUnit) {
        FOLLOWER_Y_TOLERANCE = HALDistanceUnit.convert(yTolerance, toleranceUnit, HALDistanceUnit.INCHES);
        return this;
    }

    public RoadrunnerConfig setFollowerYTolerance(double yToleranceInches) {
        setFollowerYTolerance(yToleranceInches, HALDistanceUnit.INCHES);
        return this;
    }

    public RoadrunnerConfig setFollowerHeadingTolerance(double headingTolerance, HALAngleUnit angleUnit) {
        FOLLOWER_HEADING_TOLERANCE = angleUnit.convertTo(HALAngleUnit.RADIANS).apply(headingTolerance);
        return this;
    }

    public RoadrunnerConfig setFollowerHeadingTolerance(double headingToleranceRadians) {
        setFollowerHeadingTolerance(headingToleranceRadians, HALAngleUnit.RADIANS);
        return this;
    }

    public RoadrunnerConfig setFollowerTimeout(double timeout) {
        FOLLOWER_TIMEOUT = timeout;
        return this;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
