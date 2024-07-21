package com.qualcomm.hardware.sparkfun;

/**
 * Extend SparkFunOTOS, adding methods that are to be used only internally
 */
public class SparkFunOTOSInternal extends SparkFunOTOS {

    /**
     * Update the OTOS sensor to reflect current robot Pose. Parameters passed to this method are
     * in the RAW coordinate system (origin is center of field, X-Right, Y-Up), in MKS units.
     * @param posM    Raw pose in meters, radians
     * @param velM      Raw velocity in meters/sec, radians/sec
     * @param accelM    Raw acceleration in meters/sec2, radians/sec2
     */
    public synchronized void update(Pose2D posM, Pose2D velM, Pose2D accelM){
        rawPoseMR = posM;
        rawVelMR = velM;
        rawAccelMR = accelM;
        internalUpdate();
    }

}
