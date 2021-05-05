package virtual_robot.robots.classes;

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
 * For internal use only. Represents a robot with four omni wheels, color sensor, four distance sensors,
 * a BNO055IMU, and a Continuous Rotation Servo-controlled arm on the back. Physics-Based.
 *
 * XDriveBot is the controller class for the "xdrive_bot.fxml" markup file.
 *
 */
@BotConfig(name = "XDrive Bot", filename = "xdrive_bot")
public class XDriveBot extends XDrivePhysicsBase {

    private final MotorType MOTOR_TYPE = MotorType.Neverest40;
    private DcMotorExImpl[] motors = null;
    private BNO055IMUImpl imu = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private CRServoImpl crServo = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    // backServoArm is instantiated during loading via a fx:id property
    @FXML private Rectangle backServoArm;

    public XDriveBot() {
        super();
    }

    public void initialize(){
        super.initialize();

        hardwareMap.setActive(true);

        crServo = (CRServoImpl)hardwareMap.crservo.get("back_crservo");

        hardwareMap.setActive(false);

        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
    }

    protected void createHardwareMap(){
        super.createHardwareMap();
        hardwareMap.put("back_crservo", new CRServoImpl(720));
    }

    public synchronized void updateStateAndSensors(double millis){
        super.updateStateAndSensors(millis);

        crServo.updatePositionDegrees(millis);
    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        ((Rotate)backServoArm.getTransforms().get(0)).setAngle(-crServo.getPositionDegrees());
    }

    public void powerDownAndReset(){
        super.powerDownAndReset();
    }


}
