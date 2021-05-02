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
 * a BNO055IMU, and a Continuous Rotation Servo-controlled arm on the back.
 *
 * XDriveBot is the controller class for the "xdrive_bot.fxml" markup file.
 *
 */
@BotConfig(name = "XDrive Bot", filename = "xdrive_bot")
public class XDriveBot extends VirtualBot {

    private final MotorType MOTOR_TYPE = MotorType.Neverest40;
    private DcMotorExImpl[] motors = null;
    //private VirtualRobotController.GyroSensorImpl gyro = null;
    private BNO055IMUImpl imu = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private CRServoImpl crServo = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    // backServoArm is instantiated during loading via a fx:id property
    @FXML private Rectangle backServoArm;

    private double wheelCircumference;
    private double wheelBaseRadius;

    private double[][] tWR; //Transform from wheel motion to robot motion


    public XDriveBot() {
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
        //gyro = (VirtualRobotController.GyroSensorImpl)hardwareMap.gyroSensor.get("gyro_sensor");
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");
        crServo = (CRServoImpl)hardwareMap.crservo.get("back_crservo");
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

        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
    }

    protected void createHardwareMap(){
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[] {"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (String name: motorNames) hardwareMap.put(name, new DcMotorExImpl(MOTOR_TYPE));
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        //hardwareMap.put("gyro_sensor", controller.new GyroSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("back_crservo", new CRServoImpl(720));
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

        x += dxR * cos - dyR * sin;
        y += dxR * sin + dyR * cos;
        headingRadians += headingChange;

        if (headingRadians > Math.PI) headingRadians -= 2.0 * Math.PI;
        else if (headingRadians < -Math.PI) headingRadians += 2.0 * Math.PI;

        constrainToBoundaries();

        //gyro.updateHeading(headingRadians * 180.0 / Math.PI);
        imu.updateHeadingRadians(headingRadians);

        colorSensor.updateColor(x, y);

        final double piOver2 = Math.PI / 2.0;
        for (int i = 0; i<4; i++){
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance( x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }

        crServo.updatePositionDegrees(millis);

    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        ((Rotate)backServoArm.getTransforms().get(0)).setAngle(-crServo.getPositionDegrees());
    }

    public void powerDownAndReset(){
        for (int i=0; i<4; i++) motors[i].stopAndReset();
        //gyro.deinit();
        imu.close();
    }


}
