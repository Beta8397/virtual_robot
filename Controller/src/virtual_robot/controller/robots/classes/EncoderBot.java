package virtual_robot.controller.robots.classes;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.*;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Represents a robot with four mechanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, and a Servo-controlled arm on the back.
 *
 * MechanumBot is the controller class for the "mechanum_bot.fxml" markup file.
 *
 */
@BotConfig(name = "Encoder Bot", filename = "encoder_bot")
public class EncoderBot extends VirtualBot {

    MotorType motorType;
    MotorType encoderMotorType;
    private DcMotorExImpl[] motors = null;
    //private VirtualRobotController.GyroSensorImpl gyro = null;
    private BNO055IMUImpl imu = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private ServoImpl servo = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;
    private DeadWheelEncoder rightEncoder = null;
    private DeadWheelEncoder leftEncoder = null;
    private DeadWheelEncoder xEncoder = null;

    // backServoArm is instantiated during loading via a fx:id property.
    @FXML Rectangle backServoArm;

    //Dimensions in inches for drive wheels
    private final double WHEEL_DIAMETER = 4.0;
    private final double INTER_WHEEL_WIDTH = 16.0;
    private final double INTER_WHEEL_LENGTH = 14.0;

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
    private double wheelCircumference;
    private double interWheelWidth;
    private double interWheelLength;
    private double wlAverage;
    private double encoderWheelRadius;
    private double leftEncoderX;
    private double rightEncoderX;
    private double xEncoderY;

    private double[][] tWR; //Transform from wheel motion to robot motion

    public EncoderBot(){
        super();
        motors = new DcMotorExImpl[]{
                (DcMotorExImpl)hardwareMap.get(DcMotorEx.class,"back_left_motor"),
                (DcMotorExImpl)hardwareMap.get(DcMotorEx.class,"front_left_motor"),
                (DcMotorExImpl)hardwareMap.get(DcMotorEx.class,"front_right_motor"),
                (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "back_right_motor")
        };
        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");
        servo = (ServoImpl)hardwareMap.servo.get("back_servo");
        leftEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_left");
        rightEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_right");
        xEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_x");

        //Dimensions in pixels
        wheelCircumference = Math.PI * WHEEL_DIAMETER * botWidth / 18.0;
        interWheelWidth = INTER_WHEEL_WIDTH * botWidth / 18.0;
        interWheelLength = INTER_WHEEL_LENGTH * botWidth / 18.0;
        wlAverage = (interWheelLength + interWheelWidth) / 2.0;
        encoderWheelRadius = 0.5 * ENCODER_WHEEL_DIAMETER * botWidth / 18.0;
        leftEncoderX = LEFT_ENCODER_X * botWidth / 18.0;
        rightEncoderX = RIGHT_ENCODER_X * botWidth / 18.0;
        xEncoderY = X_ENCODER_Y * botWidth / 18.0;

        tWR = new double[][] {
                {-0.25, 0.25, -0.25, 0.25},
                {0.25, 0.25, 0.25, 0.25},
                {-0.25/ wlAverage, -0.25/ wlAverage, 0.25/ wlAverage, 0.25/ wlAverage},
                {-0.25, 0.25, 0.25, -0.25}
        };
    }

    public void initialize(){
        //backServoArm = (Rectangle)displayGroup.getChildren().get(8);
        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
    }

    protected void createHardwareMap(){
        motorType = MotorType.Neverest40;
        encoderMotorType = MotorType.Neverest40;
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[] {"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (String name: motorNames) hardwareMap.put(name, new DcMotorExImpl(motorType));
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        //hardwareMap.put("gyro_sensor", controller.new GyroSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("back_servo", new ServoImpl());
        String[] encoderNames = new String[] {"enc_right", "enc_left", "enc_x"};
        for (String name: encoderNames) hardwareMap.put(name, new DeadWheelEncoder(encoderMotorType));
    }

    public synchronized void updateStateAndSensors(double millis){

        double[] deltaPos = new double[4];
        double[] w = new double[4];

        for (int i = 0; i < 4; i++) {
            deltaPos[i] = motors[i].update(millis);
            w[i] = deltaPos[i] * wheelCircumference / motorType.TICKS_PER_ROTATION;
            if (i < 2) w[i] = -w[i];
        }

        double[] robotDeltaPos = new double[] {0,0,0,0};
        for (int i=0; i<4; i++){
            for (int j = 0; j<4; j++){
                robotDeltaPos[i] += tWR[i][j] * w[j];
            }
        }

        double dxR = robotDeltaPos[0];
        double dyR = robotDeltaPos[1];
        double headingChange = robotDeltaPos[2];
        double avgHeading = headingRadians + headingChange / 2.0;

        double sin = Math.sin(avgHeading);
        double cos = Math.cos(avgHeading);

        double xOld = x;
        double yOld = y;

        x += dxR * cos - dyR * sin;
        y += dxR * sin + dyR * cos;
        headingRadians += headingChange;

        //Need to account for possibility that robot has run in to the wall
        if (x >  (halfFieldWidth - halfBotWidth)) x = halfFieldWidth - halfBotWidth;
        else if (x < (halfBotWidth - halfFieldWidth)) x = halfBotWidth - halfFieldWidth;
        if (y > (halfFieldWidth - halfBotWidth)) y = halfFieldWidth - halfBotWidth;
        else if (y < (halfBotWidth - halfFieldWidth)) y = halfBotWidth - halfFieldWidth;

        if (headingRadians > Math.PI) headingRadians -= 2.0 * Math.PI;
        else if (headingRadians < -Math.PI) headingRadians += 2.0 * Math.PI;

        imu.updateHeadingRadians(headingRadians);

        //For the deadwheel encoders, recalculate dXR and dYR to take into account the fact that the robot
        //may have run into the wall.
        double deltaX = x - xOld;
        double deltaY = y - yOld;

        dxR = deltaX * cos + deltaY * sin;
        dyR = -deltaX * sin + deltaY * cos;

        //Compute radians of rotation of each dead wheel encoder
        double rightEncoderRadians = (dyR + rightEncoderX * headingChange) / encoderWheelRadius;
        double leftEncoderRadians = -(dyR + leftEncoderX * headingChange) / encoderWheelRadius;
        double xEncoderRadians = -(dxR - xEncoderY * headingChange) / encoderWheelRadius;

        //Update positions of the dead wheel encoders
        rightEncoder.update(rightEncoderRadians, millis);
        leftEncoder.update(leftEncoderRadians, millis);
        xEncoder.update(xEncoderRadians, millis);

        colorSensor.updateColor(x, y);

        final double piOver2 = Math.PI / 2.0;

        for (int i = 0; i<4; i++){
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance( x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }

    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        ((Rotate)backServoArm.getTransforms().get(0)).setAngle(-180.0 * servo.getInternalPosition());
    }

    public void powerDownAndReset(){
        for (int i=0; i<4; i++) motors[i].stopAndReset();
        rightEncoder.stopAndReset();
        leftEncoder.stopAndReset();
        xEncoder.stopAndReset();
        imu.close();
    }


}
