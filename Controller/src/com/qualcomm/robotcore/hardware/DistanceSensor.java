package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Represents a distance sensor
 */
public interface DistanceSensor extends HardwareDevice {

    static final double distanceOutOfRange = 8200; //mm

    /**
     * Return distance of sensor from wall using specified distance unit
     * @param distanceUnit
     * @return distance
     */
    double getDistance(DistanceUnit distanceUnit);
}
