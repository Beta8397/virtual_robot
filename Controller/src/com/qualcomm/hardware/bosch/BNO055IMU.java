package com.qualcomm.hardware.bosch;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
                                      org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit);

    interface AccelerationIntegrator{}

    class CalibrationData {}

    enum AngleUnit { RADIANS, DEGREES }

    enum AccelUnit { METERS_PERSEC_PERSEC, MILLI_EARTH_GRAVITY }

}
