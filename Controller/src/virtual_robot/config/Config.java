package virtual_robot.config;

import javafx.scene.image.Image;

/**
 * Container class for the BACKGROUND image for the field.
 */
public class Config {

    //Width of Field (in pixels)
    public static final double FIELD_WIDTH = 648;

    //Whether to use "Virtual Gamepad" (true -> Virtual gamepad, false -> Real gamepad)
    public static final boolean USE_VIRTUAL_GAMEPAD = false;

    /**
     * The image object for the field.
     */
    public static final Image BACKGROUND = new Image("/virtual_robot/assets/skystone_field648.bmp");
}
