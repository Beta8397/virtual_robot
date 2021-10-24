package virtual_robot.controller;

import javafx.scene.Group;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Pane;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Transform;
import org.dyn4j.world.World;

/**
 *   For internal use only. Abstract base class for all of the specific robot configurations.
 *
 *   The VirtualBot class does not require use of the physics engine. Classes that extend
 *   VirtualBot can utilize the physics engine, or not.
 *
 *   A robot config class that extends VirtualBot must:
 *
 *   1) Provide a no-argument constructor whose first statement is super();
 *   2) Be the controller class for an .fxml file that defines the graphical respresentation of the bot;
 *   3) Provide a createHardwareMap() method;
 *   4) Provide a public synchronized updateStateAndSensors(double millis) method;
 *   5) Provide a public powerDownAndReset() method.
 *
 *   Optionally (and in most cases), it will also be necessary to:
 *
 *   1) Provide a public initialize() method for initialization of accessories whose appearance will change
 *      as the robot operates, and to set up for use of physics engine, if applicable.
 *   2) Override the public synchronized updateDisplay() method to update the appearance of accessories.
 *   3) Override the removeFromWorld() method to remove not only the chassisBody, but any additional
 *      Bodys that have been created to represent robot components.
 *
 *   The ArmBot class has detailed comments regarding the workings of these methods.
 *
 */
public abstract class VirtualBot {

    protected static VirtualRobotController controller;

    protected HardwareMap hardwareMap;

    protected Group displayGroup = null;

    // dyn4j Body, BodyFixture, and Shape for the robot chassis
    protected Body chassisBody = null;
    protected BodyFixture chassisFixture = null;
    protected org.dyn4j.geometry.Rectangle chassisRectangle = null;

    protected World<Body> world;

    protected Pane fieldPane;
    protected double halfBotWidth;
    protected double botWidth;

    /*
     * Note change: these have been made volatile, and their getters will no longer be synchronized. This is to prevent
     * a deadlock condition that was possible when trying to initialize the gyro/BNO055IMU after the INIT button is pressed.
     */
    protected volatile double x = 0;
    protected volatile double y = 0;
    protected volatile double headingRadians = 0;

    protected Translate botTranslate = null;
    protected Rotate botRotate = null;

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeadingRadians(){ return headingRadians; }

    public VirtualBot(){
        fieldPane = controller.getFieldPane();
        botWidth = VirtualField.FIELD_WIDTH / 8.0;
        halfBotWidth = botWidth / 2.0;
        world = controller.getWorld();
    }

    /**
     * Create the HardwareMap and, if using the physics engine, set up the chassis body.  Classes extending
     * VirtualBot must have an 'initialize' method which have as its first statement 'super.init()'
     */
    public void initialize(){
        createHardwareMap();
        setUpChassisBody();
    }

    static void setController(VirtualRobotController ctrl){
        controller = ctrl;
    }

    /**
     *  Set up the chassisBody and add it to the dyn4j world. This method creates a Body and adds a BodyFixture
     *  containing a Rectangle. Add the chassis body to the world.
     *
     *  Overriding this method is optional. The default do-nothing implementation is for a robot that
     *  is not controlled by the dyn4j physics engine.
     */
    public void setUpChassisBody(){ }

    /**
     * Set up the Group object that will be displayed as the virtual robot. The resource file should contain
     * a Group with a 75x75 rectangle (The chassis rectangle) as its lowest layer, and other robot components
     * on top of that rectangle.
     *
     */
    protected void setUpDisplayGroup(Group group){

        displayGroup = group;

        // This transform ensures that (x=0,y=0) corresponds to the center of the field.
        displayGroup.getTransforms().add(new Translate(VirtualField.HALF_FIELD_WIDTH - halfBotWidth,
                VirtualField.HALF_FIELD_WIDTH - halfBotWidth));

        /*
         * The following transforms will be appled in the reverse order to that in which they are added
         */

        // This will be used to display the bot at its current position (x,y)
        botTranslate = new Translate(0, 0);
        displayGroup.getTransforms().add(botTranslate);

        // This will be used to display the bot at the correct heading
        botRotate = new Rotate(0, halfBotWidth, halfBotWidth);
        displayGroup.getTransforms().add(botRotate);

        // This will adjust the bot to the correct size, based on size of the field display
        displayGroup.getTransforms().add(new Scale(botWidth/75, botWidth/75, 0, 0));

        fieldPane.getChildren().add(displayGroup);
    }

    /**
     *  Update the state of the robot. This includes the x, y, and headingRadians variables, as well other variables
     *  that may need to be updated for a specific robot configuration. This may include changes in the physics
     *  body for the VirtualBot, or changes to game elements (if a listener has flagged that the robot should
     *  take control of a game element).
     *
     *  Also, update the robot's sensors by calling the update.. methods of the sensors (e.g., the
     *  updateDistance(...) method of the distance sensors).
     *
     *  updateStateAndSensors is called on a non-UI thread via an ExecutorService object. For that reason,
     *  it SHOULD NOT make changes to the robot's graphical UI. Those changes should be made by
     *  overriding the updateDisplay() method, which is run on the UI thread.
     *
     *  @param millis milliseconds since the previous update
     */
    public abstract void updateStateAndSensors(double millis);

    /**
     *  Update the display based on the current x, y, and headingRadians of the robot.
     *  This method is run on the UI thread via a call to Platform.runLater(...).
     *
     *  For most robot configurations, it will be necessary to override this method, so as to
     *  implement graphical behavior that is specific to an individual robot configuration.
     *
     *  When overriding updateDisplay(), the first statement of the override method should
     *  be: super.updateDisplay().
     *
     */
    public synchronized void updateDisplay(){
        double displayX = x;
        double displayY = -y;
        double displayAngle = -headingRadians * 180.0 / Math.PI;
        botTranslate.setX(displayX);
        botTranslate.setY(displayY);
        botRotate.setAngle(displayAngle);
    }

    /**
     * Stop all motors; De-initialize or close other hardware (e.g. gyro/IMU) as appropriate.
     */
    public abstract void powerDownAndReset();


    /**
     * Position the VirtualBot on the field using a MouseEvent.
     * This also makes the necessary adjustment to the position of the dyn4j chassis Body,
     * and ensures that the chassis body is stationary, with no accumulated forces.
     *
     * NOTE: For a robot that includes additional Bodys attached by joints, Override this method.
     * First call can be super. Then, set the linear and angular velocities of any attached
     * Bodys to zero, and clear their accumulated forces/torques. As long a they are attached by rigid
     * joints to the chassis body, it shouldn't be necessary to explicitly position them.
     * @param arg
     */
    public synchronized void positionWithMouseClick(MouseEvent arg){

        if (arg.getButton() == MouseButton.PRIMARY) {
            x = arg.getX() - VirtualField.HALF_FIELD_WIDTH;
            y = VirtualField.HALF_FIELD_WIDTH - arg.getY();
            constrainToBoundaries();
            updateDisplay();
        }
        else if (arg.getButton() == MouseButton.SECONDARY){
            double centerX = x + VirtualField.HALF_FIELD_WIDTH;
            double centerY = VirtualField.HALF_FIELD_WIDTH - y;
            double displayAngleRads = Math.atan2(arg.getX() - centerX, centerY - arg.getY());
            headingRadians = -displayAngleRads;
            constrainToBoundaries();
            updateDisplay();
        }

        if (chassisBody != null){
            Transform t = new Transform();
            t.rotate(headingRadians);
            t.translate(x/ VirtualField.PIXELS_PER_METER, y/ VirtualField.PIXELS_PER_METER);
            chassisBody.setTransform(t);
            chassisBody.setLinearVelocity(0, 0);
            chassisBody.setAngularVelocity(0);
            chassisBody.clearAccumulatedForce();
            chassisBody.clearAccumulatedTorque();
        }

    }

    public void removeFromDisplay(Pane fieldPane){
        fieldPane.getChildren().remove(displayGroup);
    }

    /**
     * Remove chassisBody (if not null) from world. If subclass adds more Bodys, this method will need
     * to be overridden (with a call to super.removeFromWorld())
     */
    public void removeFromWorld(){
        if (chassisBody != null && world.containsBody(chassisBody)) world.removeBody(chassisBody);
    }

    public HardwareMap getHardwareMap(){ return hardwareMap; }

    /**
     * Create the HardwareMap object for the specific robot configuration, and assign it to the
     * hardwareMap variable.
     */
    protected abstract void createHardwareMap();

    /**
     * Constrain robot to the boundaries X_MIN, X_MAX, Y_MIN, Y_MAX
     * If physics simulation is being used, this should not be called during active simulation (just let robot collide
     * with walls). But, it is needed for positioning robot with mouse.
     */
    protected void constrainToBoundaries(){
        double effectiveHalfBotWidth;    //Use this to keep corner of robot from leaving field
        if (headingRadians <= -Math.PI/2.0) effectiveHalfBotWidth = -halfBotWidth * (Math.sin(headingRadians) + Math.cos(headingRadians));
        else if (headingRadians <= 0) effectiveHalfBotWidth = halfBotWidth * (Math.cos(headingRadians) - Math.sin(headingRadians));
        else if (headingRadians <= Math.PI/2.0) effectiveHalfBotWidth = halfBotWidth * (Math.sin(headingRadians) + Math.cos(headingRadians));
        else effectiveHalfBotWidth = halfBotWidth * (Math.sin(headingRadians) - Math.cos(headingRadians));

        if (x >  (VirtualField.X_MAX - effectiveHalfBotWidth)) x = VirtualField.X_MAX - effectiveHalfBotWidth;
        else if (x < (VirtualField.X_MIN + effectiveHalfBotWidth)) x = VirtualField.X_MIN + effectiveHalfBotWidth;
        if (y > (VirtualField.Y_MAX - effectiveHalfBotWidth)) y = VirtualField.Y_MAX - effectiveHalfBotWidth;
        else if (y < (VirtualField.Y_MIN + effectiveHalfBotWidth)) y = VirtualField.Y_MIN + effectiveHalfBotWidth;
    }

}
