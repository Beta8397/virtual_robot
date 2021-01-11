package system.robot.localizer;

import system.robot.subsystems.drivetrain.DriveConfig;
import util.math.units.HALDistanceUnit;

public class TrackingWheelConfig extends DriveConfig {
    public TrackingWheelConfig(double wheelRadius, HALDistanceUnit radiusUnit, double gearRatio, double encoderTicksPerRev) {
        super(wheelRadius, radiusUnit, gearRatio, 1, HALDistanceUnit.INCHES, 1, HALDistanceUnit.INCHES, encoderTicksPerRev, 1);
    }

    public TrackingWheelConfig(double wheelRadiusInches, double gearRatio, double encoderTicksPerRev) {
        this(wheelRadiusInches, HALDistanceUnit.INCHES, gearRatio, encoderTicksPerRev);
    }
}
