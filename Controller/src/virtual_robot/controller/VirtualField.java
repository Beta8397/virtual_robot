package virtual_robot.controller;

import javafx.scene.layout.StackPane;
import org.dyn4j.geometry.Vector2;
import virtual_robot.config.Config;

/**
 *   For internal use only. Class for playing field configurations.
 *
 *   Maintains the playing field dimensions for reference from both robots and game elements.
 *   Also, provides methods for converting field coordinates between pixels, meters, and inches.
 *
 */
public class VirtualField {

    public enum Unit {PIXEL, METER, INCH}

    public static final double FIELD_WIDTH = Config.FIELD_WIDTH;         // Field width in Pixels
    public static final double HALF_FIELD_WIDTH = FIELD_WIDTH / 2.0;     // Half field width in Pixels

    public static final double FIELD_WIDTH_INCHES = 144;
    public static final double HALF_FIELD_WIDTH_INCHES = FIELD_WIDTH_INCHES / 2.0;
    public static final double FIELD_WIDTH_METERS = FIELD_WIDTH_INCHES * 0.0254;
    public static final double HALF_FIELD_WIDTH_METERS = FIELD_WIDTH_METERS / 2.0;

    public static final double PIXELS_PER_METER = FIELD_WIDTH / FIELD_WIDTH_METERS;
    public static final double PIXELS_PER_INCH = FIELD_WIDTH / FIELD_WIDTH_INCHES;

    public static final double INCHES_PER_METER = 1.0 / 0.0254;

    public static final double X_MIN = 2.0 * (Config.X_MIN_FRACTION - 0.5) * HALF_FIELD_WIDTH;
    public static final double X_MAX = 2.0 * (Config.X_MAX_FRACTION - 0.5) * HALF_FIELD_WIDTH;
    public static final double Y_MIN = 2.0 * (Config.Y_MIN_FRACTION - 0.5) * HALF_FIELD_WIDTH;
    public static final double Y_MAX = 2.0 * (Config.Y_MAX_FRACTION - 0.5) * HALF_FIELD_WIDTH;

    public static double conversionFactor(Unit fromUnit, Unit toUnit){
        if (fromUnit == toUnit) return 1.0;
        switch (fromUnit){
            case PIXEL:
                return toUnit == Unit.METER? 1.0 / PIXELS_PER_METER : 1.0 / PIXELS_PER_INCH;
            case METER:
                return toUnit == Unit.PIXEL? PIXELS_PER_METER : INCHES_PER_METER;
            case INCH:
                return toUnit == Unit.PIXEL? PIXELS_PER_INCH : 1.0 / INCHES_PER_METER;
            default:
                return 1.0;
        }
    }

}
