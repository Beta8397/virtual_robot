package virtual_robot.config;

import javafx.scene.image.Image;

/**
 * Class for configuring field (width and image), and gamepad (virtual vs. real)
 */
public class Config {
    /**
     * FRACTION OF TOTAL FIELD TO CONSTRAIN ROBOT MOTION
     */
    public static final double X_MIN_FRACTION = 0; //0 for WHOLE FIELD OR BLUE REMOTE, 0.3333 for RED REMOTE FIELD
    public static final double X_MAX_FRACTION = 1; //1 for WHOLE FIELD OR RED REMOTE, 0.6667 for BLUE REMOTE FIELD
    public static final double Y_MIN_FRACTION = 0;  //Leave this alone for Ultimate Goal
    public static final double Y_MAX_FRACTION = 1;  //Leave this alone for Ultimate Goal

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
