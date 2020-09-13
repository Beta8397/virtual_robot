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
    public static final boolean USE_VIRTUAL_GAMEPAD = false;

    /**
     * The image object for the field.
     */
    public static final Image BACKGROUND = new Image("/virtual_robot/assets/ultimate_goal_648.bmp");

    /**
     * If true, Virtual Gamepad joysticks and triggers will stay in the position where they were
     * released by default. If false, the default behavior will be to "snap back" to zero.
     *
     * But if the SHIFT or ALT key is down when the control is released, the behavior will be the opposite of the
     * default behavior.
     */
    public static final boolean HOLD_CONTROLS_BY_DEFAULT = true;
}
