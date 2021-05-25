package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Vector2;
import virtual_robot.controller.*;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;
import virtual_robot.dyn4j.Hinge;
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

    // The backServoArm object is instantiated during loading via a fx:id property.
    @FXML Rectangle backServoArm;

    // dyn4j Body for the arm
    Body armBody;

    // Hinge joint for the arm
    Hinge armHinge;

    // Arm Angle in degrees
    double armAngleDegrees = 0;

    public TwoWheelBot(){
        super();
    }

    public void initialize(){
        super.initialize();
        hardwareMap.setActive(true);
        servo = (ServoImpl)hardwareMap.servo.get("back_servo");
        hardwareMap.setActive(false);

        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));

        armBody = Dyn4jUtil.createBody(backServoArm, this, 9, 9,
                new FixtureData(Filters.CHASSIS_FILTER, 1, 0, 0.25));
        world.addBody(armBody);
        armHinge = new Hinge(chassisBody, armBody, new Vector2(0, -30), VirtualField.Unit.PIXEL);
        world.addJoint(armHinge);
    }

    protected void createHardwareMap(){
        super.createHardwareMap();
        hardwareMap.put("back_servo", new ServoImpl());
    }

    public synchronized void updateStateAndSensors(double millis){
        super.updateStateAndSensors(millis);
        armAngleDegrees = -180.0 * servo.getInternalPosition();
        armHinge.setPosition(Math.toRadians(armAngleDegrees));
    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        ((Rotate)backServoArm.getTransforms().get(0)).setAngle(armAngleDegrees);
    }

    public void powerDownAndReset(){
        super.powerDownAndReset();
    }


}
