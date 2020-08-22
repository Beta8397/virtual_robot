package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PassiveDistanceSensorImpl implements DistanceSensor {

    private final double minDistMM;
    private final double maxDistMM;
    private final double lowReadingMM;
    private final double highReadingMM;

    private double distanceMM;

    /**
     * Constructor
     * @param minDistMM         Minimum distance resulting in a valid reading (mm)
     * @param maxDistMM         Maximum distance resulting in a valid reading (mm)
     * @param lowReadingMM      Reading that results when distance is less than minDistMM
     * @param highReadingMM     Reading that results when distance is greater than maxDistMM
     */
    public PassiveDistanceSensorImpl(double minDistMM, double maxDistMM, double lowReadingMM, double highReadingMM){
        this.minDistMM = minDistMM;
        this.maxDistMM = maxDistMM;
        this.lowReadingMM = lowReadingMM;
        this.highReadingMM = highReadingMM;
        distanceMM = highReadingMM;
    }

    @Override
    public synchronized double getDistance(DistanceUnit distanceUnit) {
        return distanceUnit.fromMm(distanceMM);
    }

    public synchronized void update(double distanceMM){
        if (distanceMM < minDistMM){
            this.distanceMM = lowReadingMM;
        } else if (distanceMM > maxDistMM){
            this.distanceMM = highReadingMM;
        } else {
            this.distanceMM = distanceMM;
        }
    }

}
