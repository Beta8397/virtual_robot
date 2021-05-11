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

    private CRServoImpl crServo = null;

    // backServoArm is instantiated during loading via a fx:id property
    @FXML private Rectangle backServoArm;

    private MotorType encoderMotorType;
    private DeadWheelEncoder rightEncoder = null;
    private DeadWheelEncoder leftEncoder = null;
    private DeadWheelEncoder xEncoder = null;

    //Dimensions in inches for encoder wheels.
    //Right and left encoder wheels are oriented parallel to robot-Y axis (i.e., fwd-reverse)
    //X Encoder wheel is oriented parallel to the robot-X axis (i.e., right-left axis)
    private final double ENCODER_WHEEL_DIAMETER = 2.0;
    //Distances of right and left encoder wheels from robot centerline (i.e., the robot-X coordinates of the wheels)
    private final double LEFT_ENCODER_X = -6.0;
    private final double RIGHT_ENCODER_X = 6.0;
    //Distance of X-Encoder wheel from robot-X axis (i.e., the robot-Y coordinate of the wheel)
    private final double X_ENCODER_Y = 0.0;

    //Dimensions in pixels -- to be determined in the constructor
    private double encoderWheelRadius;
    private double leftEncoderX;
    private double rightEncoderX;
    private double xEncoderY;

    public XDriveBot() {
        super();
    }

    public void initialize(){
        super.initialize();

        hardwareMap.setActive(true);

        crServo = (CRServoImpl)hardwareMap.crservo.get("back_crservo");

        leftEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_left");
        rightEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_right");
        xEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_x");

        //Dimensions in pixels
        encoderWheelRadius = 0.5 * ENCODER_WHEEL_DIAMETER * botWidth / 18.0;
        leftEncoderX = LEFT_ENCODER_X * botWidth / 18.0;
        rightEncoderX = RIGHT_ENCODER_X * botWidth / 18.0;
        xEncoderY = X_ENCODER_Y * botWidth / 18.0;

        hardwareMap.setActive(false);

        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
    }

    protected void createHardwareMap(){
        super.createHardwareMap();
        encoderMotorType = MotorType.Neverest40;
        hardwareMap.put("back_crservo", new CRServoImpl(720));
        String[] encoderNames = new String[] {"enc_right", "enc_left", "enc_x"};
        for (String name: encoderNames) hardwareMap.put(name, new DeadWheelEncoder(encoderMotorType));
    }

    public synchronized void updateStateAndSensors(double millis){
        // Save old x, y, and headingRadians values for updating free wheel encoders later
        double xOld = x;
        double yOld = y;
        double headingOld = headingRadians;

        // Compute new pose and update various sensors
        super.updateStateAndSensors(millis);

        // Update the dead wheel encoders
        double deltaX = x - xOld;
        double deltaY = y - yOld;
        double headingChange = AngleUtils.normalizeRadians(headingRadians - headingOld);
        double avgHeading = AngleUtils.normalizeRadians(headingOld + 0.5 * headingChange);
        double sin = Math.sin(avgHeading);
        double cos = Math.cos(avgHeading);

        double dxR = deltaX * cos + deltaY * sin;
        double dyR = -deltaX * sin + deltaY * cos;

        //Compute radians of rotation of each dead wheel encoder
        double rightEncoderRadians = (dyR + rightEncoderX * headingChange) / encoderWheelRadius;
        double leftEncoderRadians = -(dyR + leftEncoderX * headingChange) / encoderWheelRadius;
        double xEncoderRadians = -(dxR - xEncoderY * headingChange) / encoderWheelRadius;

        //Update positions of the dead wheel encoders
        rightEncoder.update(rightEncoderRadians, millis);
        leftEncoder.update(leftEncoderRadians, millis);
        xEncoder.update(xEncoderRadians, millis);

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
