package virtual_robot.controller;

import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.hardware.DcMotor;
import virtual_robot.hardware.HardwareMap;
import virtual_robot.util.navigation.AngleUtils;

public class TwoWheelBot extends VirtualBot {

    private VirtualRobotController.DcMotorImpl leftMotor = null;
    private VirtualRobotController.DcMotorImpl rightMotor = null;
    private VirtualRobotController.GyroSensorImpl gyro = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private VirtualRobotController.ServoImpl servo = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    private Rectangle backServoArm = null;
    private double wheelCircumference;
    private double interWheelDistance;



    public TwoWheelBot(VirtualRobotController controller){
        super(controller, "two_wheel_bot.fxml");
        leftMotor = (VirtualRobotController.DcMotorImpl)hardwareMap.dcMotor.get("left_motor");
        rightMotor = (VirtualRobotController.DcMotorImpl)hardwareMap.dcMotor.get("right_motor");
        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };
        gyro = (VirtualRobotController.GyroSensorImpl)hardwareMap.gyroSensor.get("gyro_sensor");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");
        servo = (VirtualRobotController.ServoImpl)hardwareMap.servo.get("back_servo");
        wheelCircumference = Math.PI * botWidth / 4.5;
        interWheelDistance = botWidth * 8.0 / 9.0;
        backServoArm = (Rectangle)displayGroup.getChildren().get(5);
        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
    }

    protected void createHardwareMap(){
        hardwareMap = new HardwareMap();
        hardwareMap.put("left_motor", controller.new DcMotorImpl());
        hardwareMap.put("right_motor", controller.new DcMotorImpl());
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        hardwareMap.put("gyro_sensor", controller.new GyroSensorImpl());
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("back_servo", controller.new ServoImpl());
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
        double leftWheelDist = intervalLeftTicks * wheelCircumference / VirtualRobotController.DcMotorImpl.TICKS_PER_ROTATION;
        if (leftMotor.getDirection() == DcMotor.Direction.FORWARD) leftWheelDist = -leftWheelDist;
        double rightWheelDist = intervalRightTicks * wheelCircumference / VirtualRobotController.DcMotorImpl.TICKS_PER_ROTATION;
        if (rightMotor.getDirection() == DcMotor.Direction.REVERSE) rightWheelDist = -rightWheelDist;
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
