package virtual_robot.util.navigation;

public class AngleUtils {

    public static double normalizeRadians(double radians){
        double temp = (radians + Math.PI) / (2.0 * Math.PI);
        return (temp - Math.floor(temp) - 0.5)  * 2.0 * Math.PI;
    }

    public static double normalizeDegrees(double degrees){
        double temp = (degrees + 180.0) / 360.0;
        return (temp - Math.floor(temp) - 180.0) * 360.0;
    }

    public static double normalizeDegrees360(double degrees){
        double temp = degrees / 360.0;
        return (temp - Math.floor(temp)) * 360.0;
    }

}
