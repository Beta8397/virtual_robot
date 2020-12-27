package virtual_robot.controller;

import javafx.geometry.Bounds;
import javafx.scene.Group;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import virtual_robot.controller.robots.classes.BoPBot;
import virtual_robot.util.Vector2D;

/**
 *  For internal use only. Abstract base class for game element configurations.
 *
 *  A game element class that extends VirtualGameElement must:
 *
 *  1) Provide a no-argument constructor whose first statement is super();
 *  2) Be the controller class for an .fxml file that defines the graphical representation of the game element;
 *  3) Provide a public synchronized updateState(double millis) method
 *
 *  Optionally (and in most cases), it will also be necessary to:
 *
 *  1) Provide a public initialize() method for initialization of of the game element whose appearance
 *     may change as it interacts with the robot.
 *  2) Override the public synchronized updateDisplay() method to update the appearance.
 *
 *  The Wobble Goal class has detailed comments regarding the workings of these methods.
 */
public abstract class VirtualGameElement {
    protected static VirtualRobotController controller;

    static void setController(VirtualRobotController ctrl) {
        controller = ctrl;
    }

    protected Group displayGroup = null;

    public Group getDisplayGroup() { return displayGroup; }

    protected volatile double x = 0;
    protected volatile double y = 0;

    public double getX() { return x; }
    public double getY() { return y; }

    public void setLocationInInches(Vector2D location) {
        double pixelsPerInch = controller.getField().fieldWidth / 144;
        x = location.x * pixelsPerInch;
        y = location.y * pixelsPerInch;
    }

    public void setLocation(Vector2D locationInPixels) {
        this.x = locationInPixels.x;
        this.y = locationInPixels.y;
    }

    public void setLocation(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public VirtualGameElement() {
        // TODO - figure out what goes here
    }

    /**
     * Initialize the game element.  Classes extending VirtualGameElement may override this to
     * perform custom initialization.  The default implementation does nothing.
     */
    public void initialize() {
        // Intentionally empty
    }

    /**
     * Set up the Group object that will be displayed as the virtual game element. The resource file
     * should contain a Group containing the game element's visual components.
     *
     */
    protected void setUpDisplayGroup(Group group){

        displayGroup = group;

        Bounds boundsInLocal = displayGroup.getBoundsInLocal();
        double width = boundsInLocal.getWidth();
        double height = boundsInLocal.getHeight();
        displayGroup.setTranslateX(displayGroup.getTranslateX() + controller.getField().halfFieldWidth - width / 2);
        displayGroup.setTranslateY(displayGroup.getTranslateY() + controller.getField().halfFieldWidth - height / 2);

        //Create a new display group with the 600x600 transparent rectangle as its base layer, and
        //the original display group as its upper layer.

//        displayGroup = new Group(baseRect, displayGroup);

        /*
          Add transforms. They will be applied in the opposite order from the order in which they are added.
          The scale transform scales the entire display group so that the base layer has the same width as the field,
          and the chassis rectangle (originally the 75x75 rectangle) is one-eight of the field width.
          The rotate and translate transforms are added so that they can be manipulated later, when the robot moves
          around the field.
         */
        displayGroup.getTransforms().add(new Translate(0, 0));
//        displayGroup.getTransforms().add(new Rotate(0, field.halfFieldWidth, field.halfFieldWidth));
//        displayGroup.getTransforms().add(new Scale(botWidth/75.0, botWidth/75.0, 0, 0));

        controller.getFieldPane().getChildren().add(displayGroup);
    }

    /**
     *  Update the state of the game element. This includes the x and y variables, as well other variables
     *  that may need to be updated for a specific game element configuration.
     *
     *  updateState is called on a non-UI thread via an ExecutorService object. For that reason,
     *  it SHOULD NOT make changes to the game element's graphical UI. Those changes should be made by
     *  overriding the updateDisplay() method, which is run on the UI thread.
     *
     *  @param millis milliseconds since the previous update
     */
    public abstract void updateState(double millis);

    /**
     *  Update the display based on the current x, y of the game element.
     *  This method is run on the UI thread via a call to Platform.runLater(...).
     *
     *  For most game element configurations, it will be necessary to override this method, so as to
     *  implement graphical behavior that is specific to an individual game element configuration.
     *
     *  When overriding updateDisplay(), the first statement of the override method should
     *  be: super.updateDisplay().
     *
     */
    public synchronized void updateDisplay(){
        double displayX = x;
        double displayY = -y;
        Translate translate = (Translate)displayGroup.getTransforms().get(0);
        translate.setX(displayX);
        translate.setY(displayY);
    }

    public void removeFromDisplay(StackPane fieldPane) {
        fieldPane.getChildren().remove(displayGroup);
    }

    /**
     * Gets the robot controlling the game element, if any
     * @return controlling bot or null, if not controlled
     */
    public abstract VirtualBot getControlledBy();

    /**
     * Set the robot controlling the game element.  If the bot is null, then release
     * control of the game element, if it was under control.
     * @param bot the robot controlling the game element or null, if not being controlled
     */
    public abstract void setControlledBy(VirtualBot bot);
}
