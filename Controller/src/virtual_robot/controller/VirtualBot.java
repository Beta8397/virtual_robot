package virtual_robot.controller;

import javafx.collections.ObservableList;
import javafx.fxml.FXMLLoader;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 *   For internal use only. Abstract base class for all of the specific robot configurations.
 *
 *   A robot config class that extend VirtualBot must:
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
 *      as the robot operates.
 *   2) Override the public synchronized updateDisplay() method to update the appearance of accessories.
 *
 *   The ArmBot class has detailed comments regarding the workings of these methods.
 *
 */
public abstract class VirtualBot {

    protected static VirtualRobotController controller;

    protected HardwareMap hardwareMap;

    protected Group displayGroup = null;

    protected StackPane fieldPane;
    protected double fieldWidth;
    protected double halfFieldWidth;
    protected double halfBotWidth;
    protected double botWidth;

    protected double x = 0;
    protected double y = 0;
    protected double headingRadians = 0;

    public VirtualBot(){
        fieldPane = controller.getFieldPane();
        createHardwareMap();
        this.fieldWidth = fieldPane.getPrefWidth();
        halfFieldWidth = fieldWidth / 2.0;
        botWidth = fieldWidth / 8.0;
        halfBotWidth = botWidth / 2.0;
    }

    static void setController(VirtualRobotController ctrl){
        controller = ctrl;
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
        displayGroup.getTransforms().add(new Rotate(0, halfFieldWidth, halfFieldWidth));
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

    public double getHeadingRadians(){ return headingRadians; }

    public void positionWithMouseClick(MouseEvent arg){

        if (arg.getButton() == MouseButton.PRIMARY) {
            double argX = Math.max(halfBotWidth, Math.min(fieldWidth - halfBotWidth, arg.getX()));
            double argY = Math.max(halfBotWidth, Math.min(fieldWidth - halfBotWidth, arg.getY()));
            x = argX - halfFieldWidth;
            y = halfFieldWidth - argY;
            updateDisplay();
        }
        else if (arg.getButton() == MouseButton.SECONDARY){
            double centerX = x + halfFieldWidth;
            double centerY = halfFieldWidth - y;
            double displayAngleRads = Math.atan2(arg.getX() - centerX, centerY - arg.getY());
            headingRadians = -displayAngleRads;
            updateDisplay();
        }
    }

    public void removeFromDisplay(StackPane fieldPane){
        fieldPane.getChildren().remove(displayGroup);
    }

    public HardwareMap getHardwareMap(){ return hardwareMap; }

    /**
     * Create the HardwareMap object for the specific robot configuration, and assign it to the
     * hardwareMap variable.
     */
    protected abstract void createHardwareMap();

}
