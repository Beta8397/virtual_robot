/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
Modified by FTC Team Beta 8397 for use in Virtual_Robot project
 */
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
