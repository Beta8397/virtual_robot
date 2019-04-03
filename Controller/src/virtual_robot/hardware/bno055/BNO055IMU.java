package virtual_robot.hardware.bno055;

import virtual_robot.hardware.HardwareDevice;
import virtual_robot.util.navigation.AngleUnit;
import virtual_robot.util.navigation.AxesOrder;
import virtual_robot.util.navigation.AxesReference;
import virtual_robot.util.navigation.Orientation;

public interface BNO055IMU extends HardwareDevice {


    boolean initialize(Parameters parameters);

    Parameters getParameters();

    class Parameters
    {
        public AngleUnit        angleUnit           = AngleUnit.RADIANS;
        public AccelUnit        accelUnit           = AccelUnit.METERS_PERSEC_PERSEC;
        public CalibrationData  calibrationData     = null;
        public String           calibrationDataFile = null;
        public AccelerationIntegrator accelerationIntegrationAlgorithm = null;
        public boolean          loggingEnabled      = false;
        public String           loggingTag          = "AdaFruitIMU";
    }

    void close();

    Orientation getAngularOrientation();

    Orientation getAngularOrientation(AxesReference reference, AxesOrder order,
                                      virtual_robot.util.navigation.AngleUnit angleUnit);

    interface AccelerationIntegrator{}

    class CalibrationData {}

    enum AngleUnit { RADIANS, DEGREES }

    enum AccelUnit { METERS_PERSEC_PERSEC, MILLI_EARTH_GRAVITY }

}
