package virtual_robot.util;

/**
 * For internal use only. Static functions helpful for working with angles.
 *
 * NOTE: These are not included in the FTC SDK, but can be helpful within op modes. They could
 * be copied into Android Studio.
 */
public class AngleUtils {

    /**
     * Given any input angle in radians, return its normalized value in the -pi to +pi range
     * @param radians
     * @return normalized angle in radians (-pi to pi)
     */
    public static double normalizeRadians(double radians){
        double temp = (radians + Math.PI) / (2.0 * Math.PI);
        return (temp - Math.floor(temp) - 0.5)  * 2.0 * Math.PI;
    }

    /**
     * Given any input angle in degrees, return its normalized value in the -180 to +180 range.
     * @param degrees
     * @return normalized angle in degrees (-180 to 180)
     */
    public static double normalizeDegrees(double degrees){
        double temp = (degrees + 180.0) / 360.0;
        return (temp - Math.floor(temp) - 0.5) * 360.0;
    }

    /**
     * Given any input angle in degrees, return its normalized value in the 0 to 360 degree range.
     * @param degrees
     * @return normalized angle in degrees (0 to 360)
     */
    public static double normalizeDegrees360(double degrees){
        double temp = degrees / 360.0;
        return (temp - Math.floor(temp)) * 360.0;
    }

}
