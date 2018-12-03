package virtual_robot.controller;

import javafx.scene.layout.StackPane;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.hardware.DCMotor;
import virtual_robot.hardware.HardwareMap;

public class MechanumBot extends VirtualBot {

    private VirtualRobotController.DCMotorImpl[] motors = null;
    private VirtualRobotController.GyroSensorImpl gyro = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private VirtualRobotController.ServoImpl servo = null;

    private Rectangle backServoArm = null;
    private double wheelCircumference;
    private double interWheelWidth;
    private double interWheelLength;
    private double wlSum;

    private double[][] tWR; //Transform from wheel motion to robot motion


    public MechanumBot(HardwareMap hwMap, double fieldWidth, StackPane fieldPane) {
        super(hwMap, fieldWidth);
        motors = new VirtualRobotController.DCMotorImpl[]{
                hwMap.dcMotor.get("back_left_motor"),
                hwMap.dcMotor.get("front_left_motor"),
                hwMap.dcMotor.get("front_right_motor"),
                hwMap.dcMotor.get("back_right_motor")
        };
        gyro = hwMap.gyroSensor.get("gyro_sensor");
        colorSensor = hwMap.colorSensor.get("color_sensor");
        servo = hwMap.servo.get("back_servo");
        wheelCircumference = Math.PI * botWidth / 4.5;
        interWheelWidth = botWidth * 8.0 / 9.0;
        interWheelLength = botWidth * 7.0 / 9.0;
        wlSum = interWheelLength + interWheelWidth;
        setUpDisplayGroup("mechanum_bot.fxml", fieldPane);
        backServoArm = (Rectangle)displayGroup.getChildren().get(7);
        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));

        tWR = new double[][] {
                {-0.25, 0.25, -0.25, 0.25},
                {0.25, 0.25, 0.25, 0.25},
                {-0.25/wlSum, -0.25/wlSum, 0.25/wlSum, 0.25/wlSum},
                {-0.25, 0.25, 0.25, -0.25}
        };
    }

    public synchronized void updateStateAndSensors(double millis){
        double[] intervalTicks = new double[4];
        double[] w = new double[4];

        for (int i = 0; i < 4; i++) {
            double ticks = motors[i].getCurrentPositionDouble();
            motors[i].updatePosition(millis);
            intervalTicks[i] = motors[i].getCurrentPositionDouble() - ticks;
            w[i] = intervalTicks[i] * wheelCircumference / VirtualRobotController.DCMotorImpl.TICKS_PER_ROTATION;
            if ((i < 2 && motors[i].getDirection() == DCMotor.Direction.FORWARD) ||
                    (i >= 2 && motors[i].getDirection() == DCMotor.Direction.REVERSE)) w[i] = -w[i];
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
        gyro.updateHeading(headingRadians * 180.0 / Math.PI);
        colorSensor.updateColor(x, y);
    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        ((Rotate)backServoArm.getTransforms().get(0)).setAngle(-180.0 * servo.getPosition());
    }

    public void powerDownAndReset(){
        for (int i=0; i<4; i++) motors[i].setPower(0);
        gyro.deinit();
    }


}
