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
 * For internal use only. Abstract base class for all of the specific robot configurations.
 */
public abstract class VirtualBot {

    protected HardwareMap hardwareMap;

    protected Group displayGroup = null;

    protected VirtualRobotController controller;
    protected StackPane fieldPane;
    protected double fieldWidth;
    protected double halfFieldWidth;
    protected double halfBotWidth;
    protected double botWidth;

    protected double x = 0;
    protected double y = 0;
    protected double headingRadians = 0;

    public VirtualBot(VirtualRobotController controller, String fxmlResourceName){
        this.controller = controller;
        fieldPane = controller.getFieldPane();
        createHardwareMap();
        this.fieldWidth = fieldPane.getPrefWidth();
        halfFieldWidth = fieldWidth / 2.0;
        botWidth = fieldWidth / 8.0;
        halfBotWidth = botWidth / 2.0;
        setUpDisplayGroup(fxmlResourceName);
    }

    /**
     * Set up the Group object that will be displayed as the virtual robot. The resource file should contain
     * a Group with a 75x75 rectangle (The chassis rectangle) as its lowest layer, and other robot components
     * on top of that rectangle.
     *
     * @param fxmlResourceName Resource file for the virtual robot graphic
     */
    protected void setUpDisplayGroup(String fxmlResourceName){

        try {
            displayGroup = (Group) FXMLLoader.load(getClass().getResource(fxmlResourceName));
        } catch(java.io.IOException Exc){
            System.out.println("Could not load display group for two wheel bot.");
        }

        /*
           Create a transparent 600x600 rectangle to serve as the base layer of the robot. It will go
           below the 75x75 chassis rectangle.
        */

        Rectangle baseRect = new Rectangle(0, 0, 600, 600);
        baseRect.setFill(new Color(1.0, 0.0, 1.0, 0.0));
        baseRect.setVisible(true);

        /*
          Translate the 75x75 chassis rectangle and all other children by (300 - 37.5) in X and Y, so that the
          center of the chassis rectangle will be at the same location as the center of the 600x600 base
          rectangle.
         */
        for (Node n: displayGroup.getChildren()){
            n.setTranslateX(n.getTranslateX() + 300 - 37.5);
            n.setTranslateY(n.getTranslateY() + 300 - 37.5);
        }

        // Add the 600x600 rectangle as the lowest item in the display Group, below the chassis rectangle.
        displayGroup.getChildren().add(0, baseRect);

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

    public abstract void updateStateAndSensors(double millis);

    /**
     *  Update the display based on the current x, y, and headingRadians of the robot.
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

    protected abstract void createHardwareMap();

}
