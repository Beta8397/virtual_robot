package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Represents a robot with two standard wheels, color sensor, four distance sensors,
 * a Gyro Sensor, and a Servo-controlled arm on the back.
 *
 * TwoWheelBot is the controller class for the "two_wheel_bot.fxml" markup file.
 */
@BotConfig(name = "Two Wheel Bot", filename = "two_wheel_bot")
public class TwoWheelBot extends TwoWheelPhysicsBase {

    private ServoImpl servo = null;

    //The backServoArm object is instantiated during loading via a fx:id property.
    @FXML Rectangle backServoArm;

    public TwoWheelBot(){
        super();
    }

    public void initialize(){
        super.initialize();
        hardwareMap.setActive(true);
        servo = (ServoImpl)hardwareMap.servo.get("back_servo");
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
