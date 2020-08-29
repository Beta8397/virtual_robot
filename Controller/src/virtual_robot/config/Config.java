package virtual_robot.config;

import javafx.scene.image.Image;

/**
 * Class for configuring field (width and image), and gamepad (virtual vs. real)
 */
public class Config {

    /**
     *  Width of the field, in pixels
     */
    public static final double FIELD_WIDTH = 648;

    /**
     * Whether to use "Virtual Gamepad" (true -> Virtual gamepad, false -> Real gamepad)
     */
    public static final boolean USE_VIRTUAL_GAMEPAD = true;

    /**
     * The image object for the field.
     */
    public static final Image BACKGROUND = new Image("/virtual_robot/assets/skystone_field648.bmp");

    /**
     * If true, Virtual Gamepad joysticks (and eventually triggers) will stay in the position where they were
     * released by default. If false, the default behavior will be to "snap back" to zero.
     *
     * But, when one of these controls is being manipulated, pressing the SHIFT key will toggle between "locking"
     * and "snapping back" behavior.
     */
    public static final boolean HOLD_CONTROLS_BY_DEFAULT = true;
}
