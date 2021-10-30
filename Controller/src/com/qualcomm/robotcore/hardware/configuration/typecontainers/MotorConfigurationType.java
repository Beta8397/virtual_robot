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

/* Modified by Team Beta 8397 for use in the Virtual Robot Simulator */

package com.qualcomm.robotcore.hardware.configuration.typecontainers;

import com.qualcomm.robotcore.hardware.configuration.MotorType;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * {@link MotorConfigurationType} contains the amalgamated set of information that
 * is known about a given type of motor.
 */
@SuppressWarnings("WeakerAccess")
public final class MotorConfigurationType implements Cloneable{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    private double ticksPerRev;
    private double gearing;
    private double maxRPM;
    private double achieveableMaxRPMFraction = 1.0; //In FTC SDK, this is 0.85 by default
    private Rotation orientation;

    //----------------------------------------------------------------------------------------------
    // Accessing
    //----------------------------------------------------------------------------------------------

    public double getTicksPerRev()
    {
        return ticksPerRev;
    }

    public double getAchieveableMaxTicksPerSecond()
    {
        final double encoderTicksPerRev = this.getTicksPerRev();
        final double maxRPM             = this.getMaxRPM() * this.getAchieveableMaxRPMFraction();
        final double secondsPerMinute   = 60;
        return encoderTicksPerRev * maxRPM / secondsPerMinute;
    }

    public int getAchieveableMaxTicksPerSecondRounded()
    {
        return (int)Math.round(getAchieveableMaxTicksPerSecond());
    }

    public void setTicksPerRev(double ticksPerRev)
    {
        this.ticksPerRev = ticksPerRev;
    }

    public double getGearing()
    {
        return gearing;
    }

    public void setGearing(double gearing)
    {
        this.gearing = gearing;
    }

    public double getMaxRPM()
    {
        return maxRPM;
    }

    public void setMaxRPM(double maxRPM)
    {
        this.maxRPM = maxRPM;
    }

    public double getAchieveableMaxRPMFraction()
    {
        return achieveableMaxRPMFraction;
    }

    public void setAchieveableMaxRPMFraction(double achieveableMaxRPMFraction)
    {
        this.achieveableMaxRPMFraction = achieveableMaxRPMFraction;
    }

    public Rotation getOrientation()
    {
        return orientation;
    }

    public void setOrientation(Rotation orientation)
    {
        this.orientation = orientation;
    }

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public MotorConfigurationType(MotorType motorType){
        ticksPerRev = motorType.TICKS_PER_ROTATION;
        gearing = motorType.GEARING;
        achieveableMaxRPMFraction = motorType.ACHIEVABLE_MAX_RPM_FRACTION;
        orientation = motorType.REVERSED? Rotation.CCW : Rotation.CW;

        //This may seem a little backward, but note that MotorType.MAX_TICKS_PER_SECOND refers to the
        //maximum ticks per second that can be achieved in RUN_WITH_ENCODER mode
        maxRPM = 60.0 * motorType.MAX_TICKS_PER_SECOND / (achieveableMaxRPMFraction * ticksPerRev);
    }

    public MotorConfigurationType clone()
    {
        try {
            MotorConfigurationType result = (MotorConfigurationType)super.clone();
            return result;
        }
        catch (CloneNotSupportedException e)
        {
            throw new RuntimeException("internal error: Parameters not cloneable");
        }
    }

}
