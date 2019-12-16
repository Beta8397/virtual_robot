package virtual_robot.controller;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorType;
import com.qualcomm.robotcore.hardware.ServoImpl;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Represents a robot with four mechanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, and an extensible arm with a grabber on the end.
 */
@BotConfig(name = "Arm Bot", filename = "arm_bot")
public class ArmBot extends VirtualBot {

    MotorType motorType;
    private DcMotorImpl[] motors = null;
    private DcMotorImpl armMotor = null;
    private BNO055IMUImpl imu = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    private ServoImpl handServo = null;

    @FXML private Rectangle arm;
    @FXML private Rectangle hand;
    @FXML private Rectangle leftFinger;
    @FXML private Rectangle rightFinger;

    private double armScale = 1.0;

    private double wheelCircumference;
    private double interWheelWidth;
    private double interWheelLength;
    private double wlAverage;

    private double[][] tWR; //Transform from wheel motion to robot motion


    public ArmBot() {
        super();
        motors = new DcMotorImpl[]{
                (DcMotorImpl)hardwareMap.dcMotor.get("back_left_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("front_left_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("front_right_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("back_right_motor")
        };

        armMotor = (DcMotorImpl)hardwareMap.dcMotor.get("arm_motor");

        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };

        handServo = (ServoImpl)hardwareMap.servo.get("hand_servo");

        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");

        wheelCircumference = Math.PI * botWidth / 4.5;
        interWheelWidth = botWidth * 8.0 / 9.0;
        interWheelLength = botWidth * 7.0 / 9.0;
        wlAverage = (interWheelLength + interWheelWidth) / 2.0;


        tWR = new double[][] {
                {-0.25, 0.25, -0.25, 0.25},
                {0.25, 0.25, 0.25, 0.25},
                {-0.25/ wlAverage, -0.25/ wlAverage, 0.25/ wlAverage, 0.25/ wlAverage},
                {-0.25, 0.25, 0.25, -0.25}
        };
    }

    public void initialize(){

        //arm = (Rectangle)displayGroup.getChildren().get(8);
        arm.getTransforms().add(new Scale(1.0, 1.0, 37.5, 75.0));

        //hand = (Rectangle)displayGroup.getChildren().get(9);
        hand.getTransforms().add(new Translate(0, 0));

        //leftFinger = (Rectangle)displayGroup.getChildren().get(10);
        leftFinger.getTransforms().add(new Translate(0,0));

        //rightFinger = (Rectangle)displayGroup.getChildren().get(11);
        rightFinger.getTransforms().add(new Translate(0,0));
    }

    protected void createHardwareMap(){
        motorType = MotorType.Neverest40;
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[] {"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor", "arm_motor"};
        for (String name: motorNames) hardwareMap.put(name, new DcMotorImpl(motorType));
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        //hardwareMap.put("gyro_sensor", controller.new GyroSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 175));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("hand_servo", new ServoImpl());
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

        x += dxR * cos - dyR * sin;
        y += dxR * sin + dyR * cos;
        headingRadians += headingChange;

        if (x >  (halfFieldWidth - halfBotWidth)) x = halfFieldWidth - halfBotWidth;
        else if (x < (halfBotWidth - halfFieldWidth)) x = halfBotWidth - halfFieldWidth;
        if (y > (halfFieldWidth - halfBotWidth)) y = halfFieldWidth - halfBotWidth;
        else if (y < (halfBotWidth - halfFieldWidth)) y = halfBotWidth - halfFieldWidth;

        if (headingRadians > Math.PI) headingRadians -= 2.0 * Math.PI;
        else if (headingRadians < -Math.PI) headingRadians += 2.0 * Math.PI;
        imu.updateHeadingRadians(headingRadians);

        colorSensor.updateColor(x, y);

        final double piOver2 = Math.PI / 2.0;
        for (int i = 0; i<4; i++){
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance( x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }

        Scale armScaleTransform = (Scale)arm.getTransforms().get(0);
        double armTicks = armMotor.update(millis);
        double newArmScale = armScale + armTicks / 1120.;
        if (newArmScale >= 1.0 && newArmScale <= 4){
            armScale = newArmScale;
            ((Scale)arm.getTransforms().get(0)).setY(armScale);
            ((Translate)hand.getTransforms().get(0)).setY(-40.0 * (armScale - 1.0));
            ((Translate)leftFinger.getTransforms().get(0)).setY(-40.0 * (armScale - 1.0));
            ((Translate)rightFinger.getTransforms().get(0)).setY(-40.0 * (armScale - 1.0));
        }
        double servoPos = handServo.getInternalPosition();
        ((Translate)leftFinger.getTransforms().get(0)).setX(7.5 * servoPos);
        ((Translate)rightFinger.getTransforms().get(0)).setX(-7.5 * servoPos);
    }


    public void powerDownAndReset(){
        for (int i=0; i<4; i++) motors[i].stopAndReset();
        //gyro.deinit();
        imu.close();
    }


}
