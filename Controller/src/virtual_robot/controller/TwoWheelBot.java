package virtual_robot.controller;

import javafx.scene.layout.StackPane;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.hardware.DCMotor;
import virtual_robot.hardware.HardwareMap;
import virtual_robot.util.navigation.AngleUtils;

public class TwoWheelBot extends VirtualBot {

    private VirtualRobotController.DCMotorImpl leftMotor = null;
    private VirtualRobotController.DCMotorImpl rightMotor = null;
    private VirtualRobotController.GyroSensorImpl gyro = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private VirtualRobotController.ServoImpl servo = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    private Rectangle backServoArm = null;
    private double wheelCircumference;
    private double interWheelDistance;



    public TwoWheelBot(double fieldWidth, StackPane fieldPane){
        super(fieldWidth);
        hardwareMap = VirtualRobotApplication.getControllerHandle().new HardwareMapImpl(
                new String[]{"left_motor", "right_motor"},
                new String[]{"front_distance", "left_distance", "back_distance", "right_distance"}
        );
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");
        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };
        gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        servo = hardwareMap.servo.get("back_servo");
        wheelCircumference = Math.PI * botWidth / 4.5;
        interWheelDistance = botWidth * 8.0 / 9.0;
        setUpDisplayGroup("two_wheel_bot.fxml", fieldPane);
        backServoArm = (Rectangle)displayGroup.getChildren().get(5);
        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
    }

    public synchronized void updateStateAndSensors(double millis){
        double leftTicks = leftMotor.getCurrentPositionDouble();
        double rightTicks = rightMotor.getCurrentPositionDouble();
        leftMotor.updatePosition(millis);
        rightMotor.updatePosition(millis);
        double newLeftTicks = leftMotor.getCurrentPositionDouble();
        double newRightTicks = rightMotor.getCurrentPositionDouble();
        double intervalLeftTicks = newLeftTicks - leftTicks;
        double intervalRightTicks = newRightTicks - rightTicks;
        double leftWheelDist = intervalLeftTicks * wheelCircumference / VirtualRobotController.DCMotorImpl.TICKS_PER_ROTATION;
        if (leftMotor.getDirection() == DCMotor.Direction.FORWARD) leftWheelDist = -leftWheelDist;
        double rightWheelDist = intervalRightTicks * wheelCircumference / VirtualRobotController.DCMotorImpl.TICKS_PER_ROTATION;
        if (rightMotor.getDirection() == DCMotor.Direction.REVERSE) rightWheelDist = -rightWheelDist;
        double distTraveled = (leftWheelDist + rightWheelDist) / 2.0;
        double headingChange = (rightWheelDist - leftWheelDist) / interWheelDistance;
        double deltaRobotX = -distTraveled * Math.sin(headingRadians + headingChange / 2.0);
        double deltaRobotY = distTraveled * Math.cos(headingRadians + headingChange / 2.0);
        x += deltaRobotX;
        y += deltaRobotY;
        if (x >  (halfFieldWidth - halfBotWidth)) x = halfFieldWidth - halfBotWidth;
        else if (x < (halfBotWidth - halfFieldWidth)) x = halfBotWidth - halfFieldWidth;
        if (y > (halfFieldWidth - halfBotWidth)) y = halfFieldWidth - halfBotWidth;
        else if (y < (halfBotWidth - halfFieldWidth)) y = halfBotWidth - halfFieldWidth;
        headingRadians += headingChange;
        if (headingRadians > Math.PI) headingRadians -= 2.0 * Math.PI;
        else if (headingRadians < -Math.PI) headingRadians += 2.0 * Math.PI;
        gyro.updateHeading(headingRadians * 180.0 / Math.PI);
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
        ((Rotate)backServoArm.getTransforms().get(0)).setAngle(-180.0 * servo.getPosition());
    }

    public void powerDownAndReset(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        gyro.deinit();
    }


}
