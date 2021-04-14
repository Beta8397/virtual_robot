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

    public double getPixelsPerInch() {
        return FIELD_WIDTH / FIELD_WIDTH_INCHES;
    }

    public double getPixelsPerMeter() {
        return FIELD_WIDTH / FIELD_WIDTH_METERS;
    }

    public Vector2 pixelCoordsFromMeters(double x, double y){
        return new Vector2(
                HALF_FIELD_WIDTH + x * PIXELS_PER_METER,
                HALF_FIELD_WIDTH - y * PIXELS_PER_METER
        );
    }

    public Vector2 pixelCoordsFromMeters(Vector2 meters){
        return pixelCoordsFromMeters(meters.x, meters.y);
    }

    public Vector2 pixelCoordsFromInches(double x, double y){
        return new Vector2(
                HALF_FIELD_WIDTH + x * PIXELS_PER_INCH,
                HALF_FIELD_WIDTH - y * PIXELS_PER_INCH
        );
    }

    public Vector2 pixelCoordsFromInches(Vector2 inches){
        return pixelCoordsFromInches(inches.x, inches.y);
    }

    public Vector2 meterCoordsFromPixels(double x, double y){
        return new Vector2((x - HALF_FIELD_WIDTH) / PIXELS_PER_METER, (HALF_FIELD_WIDTH - y) / PIXELS_PER_METER);
    }

    public Vector2 meterCoordsFromPixels(Vector2 pixels){
        return meterCoordsFromPixels(pixels.x, pixels.y);
    }

    public Vector2 inchCoordsFromPixels(double x, double y){
        return new Vector2( (x- HALF_FIELD_WIDTH)/ PIXELS_PER_INCH, (HALF_FIELD_WIDTH -y)/ PIXELS_PER_INCH);
    }

    public Vector2 inchCoordsFromPixels(Vector2 pixels){
        return inchCoordsFromPixels(pixels.x, pixels.y);
    }

    public Vector2 meterCoordsFromInches(double x, double y){
        return new Vector2(x * 0.0254, y * 0.0254);
    }

    public Vector2 meterCoordsFromInches(Vector2 inches){
        return meterCoordsFromInches(inches.x, inches.y);
    }

    public Vector2 inchCoordsFromMeters(double x, double y){
        return new Vector2(x/0.0254, y/0.0254);
    }

    public Vector2 inchCoordsFromMeters(Vector2 meters){
        return inchCoordsFromMeters(meters.x, meters.y);
    }

}
