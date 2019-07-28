package com.qualcomm.hardware.bosch;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * interface to simulate the FTC BNO055IMU interface.
 */
public interface BNO055IMU extends HardwareDevice {


    boolean initialize(Parameters parameters);

    Parameters getParameters();

    /**
     * Parameters for initialization of the BNO055IMU
     */
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

    /**
     * Close the IMU.
     */
    void close();

    /**
     * Get the angular orientation (as an Orientation object), using the AxesReference, AxesOrder, and AngleUnit
     * specified by the imu's Parameters object
     * @return angular orientation
     */
    Orientation getAngularOrientation();


    /**
     * Get the angular orientation (as an Orientation object), using the AxesReference, AxesOrder, and AngleUnit
     * specified by the arguments
     * @param reference axes reference
     * @param order axes order
     * @param angleUnit angle unit
     * @return angular orientation
     */
    Orientation getAngularOrientation(AxesReference reference, AxesOrder order,
                                      org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit);

    /**
     * Do-nothing interface for compatibility with FTC SDK
     */
    interface AccelerationIntegrator{}

    /**
     * Do-nothing class for compatibility with FTC SDK
     */
    class CalibrationData {}

    /**
     * Enumeration of angle units
     */
    enum AngleUnit { RADIANS, DEGREES }

    /**
     * Enumeration of acceleration units
     */
    enum AccelUnit { METERS_PERSEC_PERSEC, MILLI_EARTH_GRAVITY }

}
