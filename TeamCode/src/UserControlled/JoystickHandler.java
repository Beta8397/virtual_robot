package UserControlled;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Jeremy on 8/2/2017.
 * For more intuitive joystick variable access
 */

/*
    A class to help us set up the controllers for easy usage
 */
public class JoystickHandler {
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    private double xValue = 0;
    private double yValue = 0;
    private double angle = 0;
    private double magnitude = 0;

    private Gamepad pad;
    private int joystick;

    public JoystickHandler(Gamepad p, int j){
        pad = p;
        joystick = j;
    }

    public double x(){
        update();
        return xValue;
    }

    public double y() {
        update();
        return yValue;
    }

    public double angle() {
        update();
        return angle;
    }

    public double magnitude() {
        update();
        return magnitude;
    }

    private void update(){
        updateValues();
        calculateAngle();
        calculateMagnitude();
    }

    private void updateValues(){
        if (joystick == LEFT_JOYSTICK){
            xValue = pad.left_stick_x;
            yValue = -(pad.left_stick_y);
        }
        else {
            xValue = pad.right_stick_x;
            yValue = -(pad.right_stick_y);
        }
    }

    private void calculateAngle(){
        angle = Math.toDegrees(Math.atan2(xValue, yValue));
        if(Double.isNaN(angle)) angle = 0;
    }

    private void calculateMagnitude(){
        magnitude = Math.sqrt(Math.pow(xValue,2) + Math.pow(yValue,2));
        if(Double.isNaN(magnitude)) magnitude = 0;
    }

    public String toString(){
        return "X:" + x() + " Y:" + y() + "Angle:" + angle() + "Mag:" + magnitude();
    }
}
