package virtual_robot.hardware;

import virtual_robot.util.navigation.DistanceUnit;

public interface DistanceSensor {

    static final double distanceOutOfRange = 8200; //mm

    double getDistance(DistanceUnit distanceUnit);
}
