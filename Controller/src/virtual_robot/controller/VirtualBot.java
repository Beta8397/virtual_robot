package virtual_robot.controller;

import javafx.fxml.FXMLLoader;
import javafx.scene.Group;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.StackPane;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    protected void setUpDisplayGroup(String fxmlResourceName){
        try {
            displayGroup = (Group) FXMLLoader.load(getClass().getResource(fxmlResourceName));
        } catch(java.io.IOException Exc){
            System.out.println("Could not load display group for two wheel bot.");
        }

        displayGroup.getTransforms().add(new Translate(fieldWidth/2.0 - halfBotWidth, fieldWidth/2.0 - halfBotWidth));
        displayGroup.getTransforms().add(new Rotate(0, halfBotWidth, halfBotWidth));
        displayGroup.getTransforms().add(new Scale(botWidth/75.0, botWidth/75.0, 0, 0));

        fieldPane.getChildren().add(displayGroup);
    }

    public abstract void updateStateAndSensors(double millis);

    public synchronized void updateDisplay(){
        double displayX = halfFieldWidth + x - halfBotWidth;
        double displayY = halfFieldWidth - y - halfBotWidth;
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
