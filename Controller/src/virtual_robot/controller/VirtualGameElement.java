package virtual_robot.controller;

import javafx.application.Platform;
import javafx.geometry.Bounds;
import javafx.scene.Group;
import javafx.scene.layout.Pane;
import javafx.scene.layout.StackPane;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.World;
import virtual_robot.util.Vector2D;

/**
 *  For internal use only. Abstract base class for game element configurations.
 *
 *  A game element class that extends VirtualGameElement must:
 *
 *  1) Provide a no-argument constructor whose first statement is super();
 *  2) Be the controller class for an .fxml file that defines the graphical representation of the game element;
 *     The root element of this .fxml file should be a Group. The (0,0) position of the group will correspond
 *     to the current position of the game element, and will serve as its rotation center;
 *  3) Provide a public setUpBody method which creates/configures the dyn4j physics body (including fixture(s)),
 *     and assigns it to elementBody;
 *  4) Provide a public synchronized updateState(double millis) method;
 *
 *  Optionally, it may also be necessary to:
 *
 *  1) Provide a public initialize() method for initialization of of the game element whose appearance
 *     may change as it interacts with the robot. First statement must be super.initialize().
 *  2) Override the public synchronized updateDisplay() method to update the appearance.
 *
 *  The WobbleGoal and Ring classes have detailed comments regarding the workings of these methods.
 */
public abstract class VirtualGameElement {
    protected static VirtualRobotController controller;

    protected Group displayGroup = null;

    protected Body elementBody = null;

    protected World<Body> world = null;

    private Translate translate = null;
    private Rotate rotate = null;

    protected boolean onField = false;

    /**
     * Obtain a reference to the VirtualRobotController.
     * This must be called before creating instances of game elements.
     * @param ctrl
     */
    static void setController(VirtualRobotController ctrl) {
        controller = ctrl;
    }

    public Group getDisplayGroup() { return displayGroup; }

    // Position (pixel units, Y-up) and heading (radians) of the game element.
    protected volatile double x = 0;
    protected volatile double y = 0;
    protected volatile double headingRadians = 0;

    public VirtualGameElement() {
        world = controller.getWorld();
    }

    // Accessors of position (pixel units) and heading (radians)
    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeadingRadians() { return headingRadians; }

    /**
     * Initialize the game element.
     *
     * This method just sets up the dyn4j Body/Bodys for the game element
     *
     * Classes extending VirtualGameElement may override this to perform custom initialization; in that
     * case, the first statement should be super.initialize().
     */
    public void initialize() {  }



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
            t.setTranslation(x / VirtualField.PIXELS_PER_METER, y / VirtualField.PIXELS_PER_METER);
            t.setRotation(headingRadians);
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
            t.setTranslation(x / VirtualField.PIXELS_PER_METER, y / VirtualField.PIXELS_PER_METER);
            t.setRotation(thetaRadians);
            elementBody.setTransform(t);
            elementBody.setLinearVelocity(0, 0);
            elementBody.setAngularVelocity(0);
            elementBody.clearAccumulatedForce();
            elementBody.clearAccumulatedTorque();
        }
    }

    public void setLocation(Vector2 locationPixels, double thetaRadians){
        setLocation(locationPixels.x, locationPixels.y, thetaRadians);
    }

    public void setLocationInches(double xInches, double yInches){
        setLocation(xInches * VirtualField.PIXELS_PER_INCH, yInches * VirtualField.PIXELS_PER_INCH);
    }

    public void setLocationInches(Vector2 locationInches) {
        setLocationInches(locationInches.x, locationInches.y);
    }

    public void setLocationInches(double xInches, double yInches, double thetaRadians){
        setLocation(xInches * VirtualField.PIXELS_PER_INCH, yInches * VirtualField.PIXELS_PER_INCH, thetaRadians);
    }

    public void setLocationInches(Vector2 locationInches, double thetaRadians){
        setLocationInches(locationInches.x, locationInches.y, thetaRadians);
    }

    public void setLocationMeters(double xMeters, double yMeters){
        setLocation(xMeters * VirtualField.PIXELS_PER_METER, yMeters * VirtualField.PIXELS_PER_METER);
    }

    public void setLocationMeters(Vector2 locationMeters){
        setLocation(locationMeters.x * VirtualField.PIXELS_PER_METER, locationMeters.y * VirtualField.PIXELS_PER_METER);
    }

    /**
     * This must be called after the VirtualGameElement is created.
     * @param group
     */
    public void setUpGameElement(Group group){
        /*
         * Save reference to the game element's display group, and add the transforms that will be used to control
         * its position on the field.
         */
        setUpDisplayGroup(group);
        /*
         * Create and configure the dyn4j Body for the game elements. This method is called AFTER the call to
         * setUpDisplayGroup, so that displayGroup can be used in the setUpBody() implementation to
         * automatically generate the Body, if desired.
         */
        setUpBody();
    }

    /**
     * Set up the Group object that will be displayed as the virtual game element. The resource file
     * should contain a Group containing the game element's visual components.
     *
     * Note: The ORIGIN of Group space will serve as the position of the game element, and its center
     * of rotation. So the contents (children) of the group should be positioned accordingly within
     * the group. For example, if the group contains a single circle, that circle should have (centerX, centerY)
     * of (0,0). On the other hand, if the group contains a single rectangle (W x H), the rectangle should
     * have (x,y) of (-W/2, -H/2).
     *
     */
    public void setUpDisplayGroup(Group group){

        displayGroup = group;

        /*
         * The following transforms will be applied in the reverse order to the order in which they are added.
         */

        // This transform ensures that a game element at (x=0, y=0) will be positioned at the center of the field.
        displayGroup.getTransforms().add(new Translate(VirtualField.HALF_FIELD_WIDTH,
                VirtualField.HALF_FIELD_WIDTH));

        // Transform to position game element at its current (x,y)
        translate = new Translate(0,0);
        displayGroup.getTransforms().add(translate);

        // Transform to rotate game element to its current heading
        rotate = new Rotate(0, 0, 0);
        displayGroup.getTransforms().add(rotate);

        // This transform scales game element to correct size, based on field width
        displayGroup.getTransforms().add(new Scale(VirtualField.FIELD_WIDTH/600,
                VirtualField.FIELD_WIDTH/600, 0, 0));

    }

    /**
     *  Update the state of the game element. This includes the x, y, and headingRadians variables, as well
     *  other variables that may need to be updated for a specific game element configuration.
     *
     *  updateState is called on a non-UI thread via an ExecutorService object. For that reason,
     *  it SHOULD NOT make changes to the game element's graphical UI. Those changes should be made by
     *  overriding the updateDisplay() method, which is run on the UI thread.
     *
     *  updateState is the appropriate place to make necessary changes within the dyne4j physics world
     *  (e.g., adding or removing Bodys, setting velocities, etc.)
     *
     *  The implementation of updateState should be synchronized.
     *
     *  @param millis milliseconds since the previous update
     */
    public abstract void updateState(double millis);

    /**
     *  Update the display based on the current x, y , headingRadians of the game element.
     *  This method is usually run on the UI thread via a call to Platform.runLater(...).
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
        translate.setX(displayX);
        translate.setY(displayY);
        rotate.setAngle(displayAngle);
    }

    /**
     *  Remove the game element from the display.
     */
    public void removeFromDisplay() {
        Pane fieldPane = controller.getFieldPane();
        if (Platform.isFxApplicationThread()) {
            if (fieldPane.getChildren().contains(displayGroup)) fieldPane.getChildren().remove(displayGroup);
        } else {    // If not on application thread, changes to UI must be made via call to Platform.runLater
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
        Pane fieldPane = controller.getFieldPane();
        if (Platform.isFxApplicationThread()) {
            if (!fieldPane.getChildren().contains(displayGroup)) fieldPane.getChildren().add(displayGroup);
        } else {    // If not on application thread, changes to UI must be made via call to Platform.runLater
            Platform.runLater(new Runnable() {
                @Override
                public void run() {
                    if (!fieldPane.getChildren().contains(displayGroup)) fieldPane.getChildren().add(displayGroup);
                }
            });
        }
    }


    /**
     * Add or remove the VirtualGameElement to/from field, which includes the following actions:
     *   1) set the value of onField
     *   2) Remove body from world, or add body to world.
     *   3) Remove or add the element displayGroup from/to the display.
     *
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
     *  Set up the dyn4j Body, if any, for the game element. A physics body is not mandatory, but in the large
     *  majority of cases would be used. It should be assigned to elementBody.
     *
     *  Note: The game element could include two or more physics bodies connected by joints. In such a case,
     *  the "main" body should be assigned to elementBody.
     */
    public abstract void setUpBody();

    /**
     *  Set linear and angular speeds of all dyn4j Bodys in the game element to zero.
     *  Default implementation assumes one dyn4j Body in game element. Override if more Bodys needed.
     */
    public void stop(){
        elementBody.setLinearVelocity(0,0);
        elementBody.setAngularVelocity(0);
    }

    public Body getElementBody(){
        return elementBody;
    }

}
