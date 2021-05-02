package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.*;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;

/**
 * For internal use only. Represents a robot with four mechanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, and a Servo-controlled arm on the back.
 * <p>
 * MechanumBot is the controller class for the "mechanum_bot.fxml" markup file.
 */
@BotConfig(name = "Mechanum Bot", filename = "mechanum_bot")
public class MechanumBot extends MechanumBase {

    // backServoArm is instantiated during loading via a fx:id property.
    @FXML
    Rectangle backServoArm;
    ServoImpl servo;

    public MechanumBot() {
        super();
    }

    public void initialize() {
        super.initialize();
        hardwareMap.setActive(true);
        servo = (ServoImpl) hardwareMap.servo.get("back_servo");
        hardwareMap.setActive(false);
        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
    }

    protected void createHardwareMap(){
        super.createHardwareMap();
        hardwareMap.put("back_servo", new ServoImpl());
    }

    public synchronized void updateStateAndSensors(double millis){
        super.updateStateAndSensors(millis);
    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        ((Rotate)backServoArm.getTransforms().get(0)).setAngle(-180.0 * servo.getInternalPosition());
    }

    public void powerDownAndReset(){
        super.powerDownAndReset();
    }
}
