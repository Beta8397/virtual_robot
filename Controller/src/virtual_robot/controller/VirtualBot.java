package virtual_robot.controller;

import javafx.scene.Group;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Transform;
import org.dyn4j.world.World;
import virtual_robot.config.GameObject;

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
public abstract class VirtualBot implements GameObject {

    protected static VirtualRobotController controller;

    protected HardwareMap hardwareMap;

    protected Group displayGroup = null;

    // dyn4j Body, BodyFixture, and Shape for the robot chassis
    protected VRBody chassisBody = null;
    protected BodyFixture chassisFixture = null;
    protected org.dyn4j.geometry.Rectangle chassisRectangle = null;

    protected World<VRBody> world;

    protected StackPane fieldPane;
    protected double halfBotWidth;
    protected double botWidth;

    /*
     * Note change: these have been made volatile, and their getters will no longer be synchronized. This is to prevent
     * a deadlock condition that was possible when trying to initialize the gyro/BNO055IMU after the INIT button is pressed.
     */
    protected volatile double x = 0;
    protected volatile double y = 0;
    protected volatile double headingRadians = 0;

    protected final VirtualField FIELD;

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeadingRadians(){ return headingRadians; }

    public VirtualBot(){
        FIELD = VirtualField.getInstance();
        fieldPane = controller.getFieldPane();
        botWidth = FIELD.FIELD_WIDTH / 8.0;
        halfBotWidth = botWidth / 2.0;
        world = controller.getWorld();
    }

    /**
     * Create the HardwareMap.  Classes extending VirtualBot must have an 'initialize' method which has
     * as its first statement 'super.init()'
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
     *     The density value of 71.76 kg/m2 results in chassis mass of 15 kg.
     *     The "friction" of 0 refers to robot-game element and robot-wall friction (NOT robot-floor)
     *     The "restitution" of 0 refers to "bounce" when robot collides with wall and game elements
     *
     *     May want to change density, friction, and restitution to obtain desired behavior
     *
     *  The filter set on the chassisFixture indicates what other things the robot is capable of colliding with
     */
    public void setUpChassisBody(){
        chassisBody = new VRBody(this);
        double botWidthMeters = botWidth / FIELD.PIXELS_PER_METER;
        chassisFixture = chassisBody.addFixture(
                new org.dyn4j.geometry.Rectangle(botWidthMeters, botWidthMeters), 71.76, 0, 0);
        chassisRectangle = (org.dyn4j.geometry.Rectangle)chassisFixture.getShape();
        chassisFixture.setFilter(Filters.CHASSIS_FILTER);
        chassisBody.setMass(MassType.NORMAL);
        world.addBody(chassisBody);
    }

    /**
     * Set up the Group object that will be displayed as the virtual robot. The resource file should contain
     * a Group with a 75x75 rectangle (The chassis rectangle) as its lowest layer, and other robot components
     * on top of that rectangle.
     *
     */
    protected void setUpDisplayGroup(Group group){

        displayGroup = group;

        /*
           Create a transparent 600x600 rectangle to serve as the base layer of the robot. It will go
           below the 75x75 chassis rectangle.
        */

        Rectangle baseRect = new Rectangle(0, 0, 600, 600);
        baseRect.setFill(new Color(1.0, 0.0, 1.0, 0.0));
        baseRect.setVisible(true);

        /*
          Translate the display group by (300 - 37.5) in X and Y, so that the
          center of the chassis rectangle will be at the same location as the center of the 600x600 base
          rectangle.
         */

        displayGroup.setTranslateX(displayGroup.getTranslateX() + 300 - 37.5);
        displayGroup.setTranslateY(displayGroup.getTranslateY() + 300 - 37.5);

        //Create a new display group with the 600x600 transparent rectangle as its base layer, and
        //the original display group as its upper layer.

        displayGroup = new Group(baseRect, displayGroup);

        /*
          Add transforms. They will be applied in the opposite order from the order in which they are added.
          The scale transform scales the entire display group so that the base layer has the same width as the field,
          and the chassis rectangle (originally the 75x75 rectangle) is one-eight of the field width.
          The rotate and translate transforms are added so that they can be manipulated later, when the robot moves
          around the field.
         */
        displayGroup.getTransforms().add(new Translate(0, 0));
        displayGroup.getTransforms().add(new Rotate(0, FIELD.HALF_FIELD_WIDTH, FIELD.HALF_FIELD_WIDTH));
        displayGroup.getTransforms().add(new Scale(botWidth/75.0, botWidth/75.0, 0, 0));

        fieldPane.getChildren().add(displayGroup);
    }

    /**
     *  Update the state of the robot. This includes the x, y, and headingRadians variables, as well other variables
     *  that may need to be updated for a specific robot configuration.
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
        Translate translate = (Translate)displayGroup.getTransforms().get(0);
        translate.setX(displayX);
        translate.setY(displayY);
        ((Rotate)displayGroup.getTransforms().get(1)).setAngle(displayAngle);
    }

    /**
     * Stop all motors; De-initialize or close other hardware (e.g. gyro/IMU) as appropriate.
     */
    public abstract void powerDownAndReset();


    public synchronized void positionWithMouseClick(MouseEvent arg){

        if (arg.getButton() == MouseButton.PRIMARY) {
            x = arg.getX() - FIELD.HALF_FIELD_WIDTH;
            y = FIELD.HALF_FIELD_WIDTH - arg.getY();
            constrainToBoundaries();
            updateDisplay();
        }
        else if (arg.getButton() == MouseButton.SECONDARY){
            double centerX = x + FIELD.HALF_FIELD_WIDTH;
            double centerY = FIELD.HALF_FIELD_WIDTH - y;
            double displayAngleRads = Math.atan2(arg.getX() - centerX, centerY - arg.getY());
            headingRadians = -displayAngleRads;
            constrainToBoundaries();
            updateDisplay();
        }

        if (chassisBody != null){
            Transform t = new Transform();
            t.rotate(headingRadians);
            t.translate(x/ FIELD.PIXELS_PER_METER, y/ FIELD.PIXELS_PER_METER);
        }

    }

    public void removeFromDisplay(StackPane fieldPane){
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

        if (x >  (FIELD.X_MAX - effectiveHalfBotWidth)) x = FIELD.X_MAX - effectiveHalfBotWidth;
        else if (x < (FIELD.X_MIN + effectiveHalfBotWidth)) x = FIELD.X_MIN + effectiveHalfBotWidth;
        if (y > (FIELD.Y_MAX - effectiveHalfBotWidth)) y = FIELD.Y_MAX - effectiveHalfBotWidth;
        else if (y < (FIELD.Y_MIN + effectiveHalfBotWidth)) y = FIELD.Y_MIN + effectiveHalfBotWidth;
    }

}
