package virtual_robot.controller.robots.classes;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.DeadWheelEncoder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;
import virtual_robot.util.Vector2D;

/**
 * For internal use only. Represents a robot with four omni wheels, color sensor, four distance sensors,
 * a BNO055IMU, and a Continuous Rotation Servo-controlled arm on the back.
 *
 * XDriveBot is the controller class for the "xdrive_bot.fxml" markup file.
 *
 */
//@BotConfig(name = "OdomXDrive Bot", filename = "odomxdrive_bot")
public class OdomXDriveBot extends VirtualBot {

    private final MotorType MOTOR_TYPE = MotorType.Neverest40;
    public DcMotorExImpl[] motors = null;
    //private VirtualRobotController.GyroSensorImpl gyro = null;
    public BNO055IMUImpl imu = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    public VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    // backServoArm is instantiated during loading via a fx:id property


    private double wheelCircumference;
    private double wheelBaseRadius;

    private final double ENCODER_WHEEL_DIAMETER = 1.5;
    //Distances of right and left encoder wheels from robot centerline (i.e., the robot-X coordinates of the wheels)
    private final double Y_ENCODER_Y = -6.0;
    //Distance of X-Encoder wheel from robot-X axis (i.e., the robot-Y coordinate of the wheel)
    private final double X_ENCODER_X = 6.0;
    private double yEncoderY;
    private double xEncoderX;
    private double encoderWheelRadius;
    private final double ENCODER_TICKS_PER_REVOLUTION = 1120;
    private final double ENCODER_WHEEL_CIRCUMFERENCE = Math.PI * ENCODER_WHEEL_DIAMETER;
    private final double ticksToRadian = 2 * Math.PI / ENCODER_TICKS_PER_REVOLUTION;
    private final double ticksPerPixel = ENCODER_TICKS_PER_REVOLUTION / ENCODER_WHEEL_CIRCUMFERENCE / (botWidth / 18);
    public double[] pose = new double[2];
    public int[] prevTicks = new int[3];

    private double[][] tWR; //Transform from wheel motion to robot motion
    public DeadWheelEncoder[] odomEncoders;

    public OdomXDriveBot() {
        super();
    }

    public void initialize(){
        super.initialize();


        hardwareMap.setActive(true);
        motors = new DcMotorExImpl[]{
                (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "back_left_motor"),
                (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "front_left_motor"),
                (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "front_right_motor"),
                (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "back_right_motor")
        };
        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };
        odomEncoders = new DeadWheelEncoder[]{
                hardwareMap.get(DeadWheelEncoder.class, "y_enc"),
                hardwareMap.get(DeadWheelEncoder.class, "x_enc")
        };
        //gyro = (VirtualRobotController.GyroSensorImpl)hardwareMap.gyroSensor.get("gyro_sensor");
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        wheelCircumference = Math.PI * botWidth / 4.5;
        double sqrt2 = Math.sqrt(2);
        wheelBaseRadius = botWidth * (1.0/sqrt2 - 5.0/36.0);

        tWR = new double[][] {
                {-0.25*sqrt2, 0.25*sqrt2, -0.25*sqrt2, 0.25*sqrt2},
                {0.25*sqrt2, 0.25*sqrt2, 0.25*sqrt2, 0.25*sqrt2},
                {-0.25/ wheelBaseRadius, -0.25/ wheelBaseRadius, 0.25/ wheelBaseRadius, 0.25/ wheelBaseRadius},
                {-0.25, 0.25, 0.25, -0.25}
        };
        hardwareMap.setActive(false);


        encoderWheelRadius = 0.5 * ENCODER_WHEEL_DIAMETER * botWidth / 18.0;
        yEncoderY = Y_ENCODER_Y * botWidth / 18.0;
        xEncoderX = X_ENCODER_X * botWidth / 18.0;
    }

    protected void createHardwareMap(){
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[] {"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (String name: motorNames) hardwareMap.put(name, new DcMotorExImpl(MOTOR_TYPE));
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        //hardwareMap.put("gyro_sensor", controller.new GyroSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("y_enc", new DeadWheelEncoder(MotorType.Neverest40));
        hardwareMap.put("x_enc", new DeadWheelEncoder(MotorType.Neverest40));
    }

    public synchronized void updateStateAndSensors(double millis){
        double[] deltaPos = new double[4];
        double[] w = new double[4];

        for (int i = 0; i < 4; i++) {
            deltaPos[i] = motors[i].update(millis);
            w[i] = deltaPos[i] * wheelCircumference / MOTOR_TYPE.TICKS_PER_ROTATION;
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

        if (headingRadians > Math.PI) headingRadians -= 2.0 * Math.PI;
        else if (headingRadians < -Math.PI) headingRadians += 2.0 * Math.PI;

        constrainToBoundaries();

        //gyro.updateHeading(headingRadians * 180.0 / Math.PI);
        imu.updateHeadingRadians(headingRadians);



        final double piOver2 = Math.PI / 2.0;
        for (int i = 0; i<4; i++){
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance( x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }

        double deltaX = x - xOld;
        double deltaY = y - yOld;

//
//        dxR = deltaX * cos + deltaY * sin;
//        dyR = -deltaX * sin + deltaY * cos;

        Vector2D df = new Vector2D(deltaX, deltaY);
        double heading = headingRadians;
        if (heading < 0.0) {
            heading += Math.PI * 2.0;
        }
        Vector2D dr = df.rotated(-heading);
        dxR = dr.x * ticksPerPixel;
        dyR = dr.y * ticksPerPixel;


        //Compute radians of rotation of each dead wheel encoder
        double yEncoderRadians = -(dyR * ticksToRadian);
        double xEncoderRadians = -(dxR * ticksToRadian);

        odomEncoders[0].update(yEncoderRadians, millis);
        odomEncoders[1].update(xEncoderRadians, millis);
    }



    public synchronized void updateDisplay(){
        super.updateDisplay();
    }

    public void powerDownAndReset(){
        for (int i=0; i<4; i++) motors[i].stopAndReset();
        //gyro.deinit();
        imu.close();
    }


}
