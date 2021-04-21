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

    private static VirtualField instance = null;

    public final double FIELD_WIDTH;         // Field width in Pixels
    public final double HALF_FIELD_WIDTH;     // Half field width in Pixels

    public final double PIXELS_PER_METER;
    public final double PIXELS_PER_INCH;

    public static final double FIELD_WIDTH_INCHES = 144;
    public static final double HALF_FIELD_WIDTH_INCHES = FIELD_WIDTH_INCHES / 2.0;
    public static final double FIELD_WIDTH_METERS = FIELD_WIDTH_INCHES * 0.0254;
    public static final double HALF_FIELD_WIDTH_METERS = FIELD_WIDTH_METERS / 2.0;

    public static final double INCHES_PER_METER = 1.0 / 0.0254;

    public final double X_MIN, X_MAX, Y_MIN, Y_MAX;     // Field boundaries in Pixels

    private VirtualField() {
        FIELD_WIDTH = Config.FIELD_WIDTH;
        HALF_FIELD_WIDTH = FIELD_WIDTH / 2.0;
        X_MIN = 2.0 * (Config.X_MIN_FRACTION - 0.5) * HALF_FIELD_WIDTH;
        X_MAX = 2.0 * (Config.X_MAX_FRACTION - 0.5) * HALF_FIELD_WIDTH;
        Y_MIN = 2.0 * (Config.Y_MIN_FRACTION - 0.5) * HALF_FIELD_WIDTH;
        Y_MAX = 2.0 * (Config.Y_MAX_FRACTION - 0.5) * HALF_FIELD_WIDTH;
        PIXELS_PER_METER = FIELD_WIDTH / FIELD_WIDTH_METERS;
        PIXELS_PER_INCH = FIELD_WIDTH / FIELD_WIDTH_INCHES;
    }

    public static VirtualField getInstance(){
        if (instance == null) {
            instance = new VirtualField();
        }
        return instance;
    }

}