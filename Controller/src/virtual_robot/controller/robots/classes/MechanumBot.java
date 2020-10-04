package virtual_robot.controller.robots.classes;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

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
        hardwareMap.setActive(true);
        servo = (ServoImpl) hardwareMap.servo.get("back_servo");
        hardwareMap.setActive(false);
    }

    public void initialize() {
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
