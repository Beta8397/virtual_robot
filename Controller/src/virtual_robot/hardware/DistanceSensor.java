package virtual_robot.hardware;

import virtual_robot.util.navigation.DistanceUnit;

public interface DistanceSensor extends HardwareDevice {

    static final double distanceOutOfRange = 8200; //mm

    double getDistance(DistanceUnit distanceUnit);
}
