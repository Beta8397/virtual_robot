package virtual_robot.controller;

import javafx.scene.layout.StackPane;
import virtual_robot.config.Config;

/**
 *   For internal use only. Class for playing field configurations.
 *
 *   Maintains the playing field dimensions for reference from both robots and game elements.
 *
 */
public class VirtualField {
    protected static VirtualRobotController controller;

    static void setController(VirtualRobotController ctrl) {
        controller = ctrl;
    }

    protected StackPane fieldPane;
    public double fieldWidth;
    public double halfFieldWidth;

    public final double X_MIN, X_MAX, Y_MIN, Y_MAX;

    public VirtualField() {
        fieldPane = controller.getFieldPane();
        fieldWidth = fieldPane.getPrefWidth();
        halfFieldWidth = fieldWidth / 2.0;
        X_MIN = 2.0 * (Config.X_MIN_FRACTION - 0.5) * halfFieldWidth;
        X_MAX = 2.0 * (Config.X_MAX_FRACTION - 0.5) * halfFieldWidth;
        Y_MIN = 2.0 * (Config.Y_MIN_FRACTION - 0.5) * halfFieldWidth;
        Y_MAX = 2.0 * (Config.Y_MAX_FRACTION - 0.5) * halfFieldWidth;
    }

    public double getPixelsPerInch() {
        return fieldWidth / 144;
    }
}
