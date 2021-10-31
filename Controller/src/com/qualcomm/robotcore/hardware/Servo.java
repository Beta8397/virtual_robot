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
Modified by FTC Team Beta 8397 for use in the Virtual_Robot Simulator
 */

package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Provides an subset of the functionality of the Servo interface in the FTC SDK.
 */
public interface Servo extends HardwareDevice {

    /**
     * The minimum and maximum positions to which a servo can be moved.
     */
    double MIN_POSITION = 0.0;
    double MAX_POSITION = 1.0;

    /**
     * Direction enum to allow software to internally reverse the values to which position is set.
     */
    enum Direction {FORWARD, REVERSE}

    /**
     * Method to set the direction
     * @param direction
     */
    void setDirection(Direction direction);

    /**
     * Method to get the direction
     * @return
     */
    Direction getDirection();

    /**
     * Set servo position
     * @param position Must be between 0 and 1
     */
    void setPosition(double position);

    /**
     * Get servo position
     * @return Current servo position
     */
    double getPosition();

    /**
     * Scale range to a pair of values between 0 and 1. After scaling the range, calls to setPosition
     * will still accept values between 0 and 1; getPosition will still return values between 0 and 1.
     * But the true position of the servo will be scaled into the range specified in the call to
     * scaleRange.
     */
    void scaleRange(double min, double max);

    default ServoController getController(){ return ServoControllerImpl.getInstance(); }

    default int getPortNumber() { return 0; }

}
