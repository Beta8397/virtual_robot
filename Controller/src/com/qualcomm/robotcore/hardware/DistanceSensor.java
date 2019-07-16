package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public interface DistanceSensor extends HardwareDevice {

    static final double distanceOutOfRange = 8200; //mm

    double getDistance(DistanceUnit distanceUnit);
}
