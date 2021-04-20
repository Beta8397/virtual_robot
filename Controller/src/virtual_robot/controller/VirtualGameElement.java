package virtual_robot.controller;

import javafx.application.Platform;
import javafx.geometry.Bounds;
import javafx.scene.Group;
import javafx.scene.layout.StackPane;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import org.dyn4j.geometry.Transform;
import org.dyn4j.world.World;
import virtual_robot.config.GameObject;
import virtual_robot.util.Vector2D;

/**
 *  For internal use only. Abstract base class for game element configurations.
 *
 *  A game element class that extends VirtualGameElement must:
 *
 *  1) Provide a no-argument constructor whose first statement is super();
 *  2) Be the controller class for an .fxml file that defines the graphical representation of the game element;
 *  3) Provide a public setUpBody method which creates/configures the dyn4j physics body (including fixture(s)),
 *     and assigns it to elementBody ;
 *  4) Provide a public synchronized updateState(double millis) method;
 *
 *  Optionally, it may also be necessary to:
 *
 *  1) Provide a public initialize() method for initialization of of the game element whose appearance
 *     may change as it interacts with the robot. First statement must be super.initialize().
 *  2) Override the public synchronized updateDisplay() method to update the appearance.
 *  3) Override the setLocation methods. This would only be necessary if the game element has
 *     more than one physics body.
 *
 *  The Wobble Goal class has detailed comments regarding the workings of these methods.
 */
public abstract class VirtualGameElement implements GameObject {
    protected static VirtualRobotController controller;

    protected Group displayGroup = null;

    protected VRBody elementBody = null;

    protected World<VRBody> world = null;

    protected final VirtualField FIELD;

    static void setController(VirtualRobotController ctrl) {
        controller = ctrl;
    }

    public Group getDisplayGroup() { return displayGroup; }

    protected volatile double x = 0;
    protected volatile double y = 0;
    protected volatile double headingRadians = 0;
    private boolean onField = false;

    public VirtualGameElement() {
        FIELD = VirtualField.getInstance();
        world = controller.getWorld();
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeadingRadians() { return headingRadians; }

    /**
     * Initialize the game element.  Classes extending VirtualGameElement may override this to
     * perform custom initialization.  The default implementation does nothing.
     */
    public void initialize() {
        setUpBody();
    }

    /**
     * Add or remove the game element from the field, which includes the following actions:
     *   1) set the value of onField
     *   2) Remove elementBody from world, or add elementBody to world.
     *   3) Remove or add the element displayGroup from/to the display.
     * @param onField
     */
    public void setOnField(boolean onField){
        this.onField = onField;
        if (onField && !world.containsBody(elementBody)) world.addBody(elementBody);
        else if (!onField && world.containsBody(elementBody)) world.removeBody(elementBody);
        if (onField) addToDisplay();
        else removeFromDisplay();
    }

    public boolean isOnField(){
        return this.onField;
    }

    /**
     * Get the location of the game element in pixel units.
     * @return
     */
    public Vector2D getLocation() {
        return new Vector2D(x, y);
    }

    /**
     * Set location of the game elements in pixel units. Besides setting the x and y fields, it is also
     * necessary to: set the location (in meters) of the elementBody (if any), set the angular and linear
     * velocity of the elementBody to 0, and clear accumulated force and torque on the elementBody.
     *
     * @param xPixels
     * @param yPixels
     */
    public void setLocation(double xPixels, double yPixels){
        this.x = xPixels;
        this.y = yPixels;
        if (elementBody != null){
            Transform t = new Transform();
            t.translate(x / FIELD.PIXELS_PER_METER, y / FIELD.PIXELS_PER_METER);
            t.rotate(headingRadians);
            elementBody.setTransform(t);
            elementBody.setLinearVelocity(0,0);
            elementBody.setAngularVelocity(0);
            elementBody.clearAccumulatedForce();
            elementBody.clearAccumulatedTorque();
        }
    }

    /**
     * Set location of the game element in pixel units
     * @param locationPixels
     */
    public void setLocation(Vector2D locationPixels){
        setLocation(locationPixels.x, locationPixels.y);
    }

    /**
     * Set location and heading of the game elements in pixel units and radians. Besides setting the x, y,
     * and headingRadians fields, it is also necessary to: set the location (in meters) of the elementBody (if any),
     * set the angular and linear velocity of the elementBody to 0, and clear accumulated force and torque on the
     * elementBody.
     *
     * @param xPixels
     * @param yPixels
     * @param thetaRadians
     */
    public void setLocation(double xPixels, double yPixels, double thetaRadians){
        this.x = xPixels;
        this.y = yPixels;
        if (elementBody != null) {
            Transform t = new Transform();
            t.translate(x / FIELD.PIXELS_PER_METER, y / FIELD.PIXELS_PER_METER);
            t.rotate(thetaRadians);
            elementBody.setTransform(t);
            elementBody.setLinearVelocity(0, 0);
            elementBody.setAngularVelocity(0);
            elementBody.clearAccumulatedForce();
            elementBody.clearAccumulatedTorque();
        }
    }

    public void setLocation(Vector2D locationPixels, double thetaRadians){
        setLocation(locationPixels.x, locationPixels.y, thetaRadians);
    }

    public void setLocationInches(double xInches, double yInches){
        setLocation(xInches * FIELD.PIXELS_PER_INCH, yInches * FIELD.PIXELS_PER_INCH);
    }

    public void setLocationInches(Vector2D locationInches) {
        setLocationInches(locationInches.x, locationInches.y);
    }

    public void setLocationInches(double xInches, double yInches, double thetaRadians){
        setLocation(xInches * FIELD.PIXELS_PER_INCH, yInches * FIELD.PIXELS_PER_INCH, thetaRadians);
    }

    public void setLocationInches(Vector2D locationInches, double thetaRadians){
        setLocationInches(locationInches.x, locationInches.y, thetaRadians);
    }

    /**
     * Set up the Group object that will be displayed as the virtual game element. The resource file
     * should contain a Group containing the game element's visual components.
     *
     */
    public void setUpDisplayGroup(Group group){

        displayGroup = group;

        Bounds boundsInLocal = displayGroup.getBoundsInLocal();
        double width = boundsInLocal.getWidth();
        double height = boundsInLocal.getHeight();
        displayGroup.setTranslateX(displayGroup.getTranslateX() + VirtualField.getInstance().HALF_FIELD_WIDTH - width / 2);
        displayGroup.setTranslateY(displayGroup.getTranslateY() + VirtualField.getInstance().HALF_FIELD_WIDTH - height / 2);

        /*
          Add transforms. They will be applied in the opposite order from the order in which they are added. The
          translate transform adjusts size of the displayed game element based on Field Width.
          The rotate and translate transforms are added so that they can be manipulated later, when the robot moves
          around the field.
         */
        displayGroup.getTransforms().add(new Translate(0, 0));
        displayGroup.getTransforms().add(new Rotate(0, VirtualField.getInstance().HALF_FIELD_WIDTH,
                VirtualField.getInstance().HALF_FIELD_WIDTH));
        displayGroup.getTransforms().add(new Scale(VirtualField.getInstance().FIELD_WIDTH/600,
                VirtualField.getInstance().FIELD_WIDTH/600, 0, 0));
    }

    /**
     *  Update the state of the game element. This includes the x and y variables, as well other variables
     *  that may need to be updated for a specific game element configuration.
     *
     *  updateState is called on a non-UI thread via an ExecutorService object. For that reason,
     *  it SHOULD NOT make changes to the game element's graphical UI. Those changes should be made by
     *  overriding the updateDisplay() method, which is run on the UI thread.
     *
     *  The implementation of updateState should be synchronized.
     *
     *  @param millis milliseconds since the previous update
     */
    public abstract void updateState(double millis);

    /**
     *  Update the display based on the current x, y of the game element.
     *  This method is run on the UI thread via a call to Platform.runLater(...).
     *
     *  For some game element configurations, it may be necessary to override this method, so as to
     *  implement graphical behavior that is specific to an individual game element configuration.
     *
     *  When overriding updateDisplay(), the first statement of the override method should
     *  be: super.updateDisplay().
     *
     */
    public synchronized void updateDisplay(){
        double displayX = x;
        double displayY = -y;
        double displayAngle = -headingRadians * 180.0 / Math.PI;
        Translate translate = (Translate)displayGroup.getTransforms().get(0);
        translate.setX(displayX);
        translate.setY(displayY);
        ((Rotate)displayGroup.getTransforms().get(1)).setAngle(displayAngle);
    }

    /**
     *  Remove the game element from the display.
     */
    public void removeFromDisplay() {
        StackPane fieldPane = controller.getFieldPane();
        if (Platform.isFxApplicationThread()) {
            if (fieldPane.getChildren().contains(displayGroup)) fieldPane.getChildren().remove(displayGroup);
        } else {
            Platform.runLater(new Runnable() {
                @Override
                public void run() {
                    if (fieldPane.getChildren().contains(displayGroup)) fieldPane.getChildren().remove(displayGroup);
                }
            });
        }
    }

    /**
     *  Add the game element to the display.
     */
    public void addToDisplay() {
        StackPane fieldPane = controller.getFieldPane();
        if (Platform.isFxApplicationThread()) {
            if (!fieldPane.getChildren().contains(displayGroup)) fieldPane.getChildren().add(displayGroup);
        } else {
            Platform.runLater(new Runnable() {
                @Override
                public void run() {
                    if (!fieldPane.getChildren().contains(displayGroup)) fieldPane.getChildren().add(displayGroup);
                }
            });
        }
    }

    /**
     *  Set up the dyn4j Body, if any, for the game element. A physics body is not mandatory, but in the large
     *  majority of cases would be used. It should be assigned to elementBody.
     *
     *  Note: The game element could include two or more physics bodies connected by joints. In such a case,
     *  the "main" body should be assigned to elementBody.
     */
    public abstract void setUpBody();

}
