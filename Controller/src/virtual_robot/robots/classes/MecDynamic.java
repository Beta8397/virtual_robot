package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.DeadWheelEncoder;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Represents a robot with four mecanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, a Servo-controlled arm on the back, and three dead-wheel encoder pods
 *
 * MecanumBot is the controller class for the "mecanum_bot.fxml" markup file.
 *
 */
@BotConfig(name = "MecDynamic Bot", filename = "mec_dynamic_bot")
public class MecDynamic extends DynamicMecanumBase {

    private ServoImpl servo = null;

    // backServoArm is instantiated during loading via a fx:id property.
    @FXML Rectangle backServoArm;

    public MecDynamic(){
        super(MotorType.Gobilda192);
    }

    public void initialize(){
        System.out.println("Initializing MecDynamic Robot");
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
        //Compute new pose and update various sensors
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
