package system.robot.subsystems.drivetrain;

import util.math.units.HALDistanceUnit;

public class DriveConfig {
    /*
     * These are motor constants that should be listed online for your motors.
     */
    public final double TICKS_PER_REV;
    public final double MAX_RPM;

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public final double WHEEL_RADIUS; // in
    public final double GEAR_RATIO; // output (wheel) speed / input (motor) speed
    public final double TRACK_WIDTH; // in
    public final double WHEEL_BASE; //in

    public DriveConfig(double wheelRadius, HALDistanceUnit wheelRadiusUnit, double gearRatio, double trackWidth, HALDistanceUnit trackWidthUnit, double wheelBase, HALDistanceUnit wheelBaseUnit, double encoderTicksPerRev, double motorMaxRPM) {
        WHEEL_RADIUS = HALDistanceUnit.convert(wheelRadius, wheelRadiusUnit, HALDistanceUnit.INCHES);
        GEAR_RATIO = gearRatio;
        TRACK_WIDTH = HALDistanceUnit.convert(trackWidth, trackWidthUnit, HALDistanceUnit.INCHES);
        WHEEL_BASE = HALDistanceUnit.convert(wheelBase, wheelBaseUnit, HALDistanceUnit.INCHES);
        TICKS_PER_REV = encoderTicksPerRev;
        MAX_RPM = motorMaxRPM;
    }

    public DriveConfig(double wheelRadiusInches, double gearRatio, double trackWidthInches, double wheelBaseInches, double motorTicksPerRev, double motorMaxRPM) {
        this(wheelRadiusInches, HALDistanceUnit.INCHES, gearRatio, trackWidthInches, HALDistanceUnit.INCHES, wheelBaseInches, HALDistanceUnit.INCHES, motorTicksPerRev, motorMaxRPM);
    }

    public DriveConfig(double wheelRadiusInches, double gearRatio, double trackWidthInches, double motorTicksPerRev, double motorMaxRPM) {
        this(wheelRadiusInches, HALDistanceUnit.INCHES, gearRatio, trackWidthInches, HALDistanceUnit.INCHES, trackWidthInches, HALDistanceUnit.INCHES, motorTicksPerRev, motorMaxRPM);
    }

    public DriveConfig(double wheelRadius, HALDistanceUnit radiusUnit, double gearRatio, double encoderTicksPerRev, double motorMaxRPM) {
        this(wheelRadius, radiusUnit, gearRatio, 1, HALDistanceUnit.INCHES, 1, HALDistanceUnit.INCHES, encoderTicksPerRev, motorMaxRPM);
    }

    public DriveConfig(double wheelRadiusInches, double gearRatio, double encoderTicksPerRev, double motorMaxRPM) {
        this(wheelRadiusInches, HALDistanceUnit.INCHES, gearRatio, encoderTicksPerRev, motorMaxRPM);
    }



    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }
}
