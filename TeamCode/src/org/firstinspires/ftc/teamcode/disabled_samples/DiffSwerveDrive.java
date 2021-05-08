package org.firstinspires.ftc.teamcode.disabled_samples;

import android.graphics.PointF;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class DiffSwerveDrive {

    public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    public static final double TICKS_PER_ROTATION = 560;
    public static final double MAX_TICKS_PER_SECOND = 2500;

    public static final double CHASSIS_RAD_SQUARED = 7*7;
    public static final double MAX_DRIVE_SPEED = MAX_TICKS_PER_SECOND * WHEEL_CIRCUMFERENCE / TICKS_PER_ROTATION;
    public static final double MAX_ANGULAR_SPEED = MAX_DRIVE_SPEED / Math.sqrt(CHASSIS_RAD_SQUARED);

    DcMotor[] motors = new DcMotor[4];      //Drive motors

    //Positions of the four drive wheels in robot-coordinate system, in inches
    public static final PointF[] WHEEL_POS = new PointF[]{
            new PointF(-6, 0),          // Left
            new PointF(6, 0)            // Right
    };


    public void init(HardwareMap hardwareMap){
        String[] motorNames = new String[] {"bottom_left_motor", "top_left_motor", "bottom_right_motor", "top_right_motor"};
        for (int i=0; i<4; i++) {
            motors[i] = (DcMotor)hardwareMap.get(DcMotor.class, motorNames[i]);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Sets the robot drive speed (linear and angular)
     * @param vx    Speed (inches per sec) in X direction (positive is rightward)
     * @param vy    Speed (inches per sec) in Y direction (positive is forward)
     * @param va    Angular speed (radians per sec)   (positive is counter-clockwise)
     */
    public void setDriveSpeed(float vx, float vy, float va) {

        double[] motorPowers = new double[4];


        for (int i = 0; i < 2; i++) {
            double wheelSpeed = Math.sqrt(vx * vx + vy * vy + va * va * CHASSIS_RAD_SQUARED
                    + 2 * va * (vy * WHEEL_POS[i].x - vx * WHEEL_POS[i].y));
            double targetSteer = normalizeRadians(Math.atan2(vy + va * WHEEL_POS[i].x, vx - va * WHEEL_POS[i].y) - Math.PI / 2);
            double currentSteer = normalizeRadians(Math.PI * (motors[2*i].getCurrentPosition() + motors[2*i+1].getCurrentPosition())
                    / (4.0 * TICKS_PER_ROTATION));
            double offset = normalizeRadians(targetSteer - currentSteer);
            boolean reversed = false;
            if (Math.abs(offset) > Math.PI / 2) {
                reversed = true;
                offset = normalizeRadians(offset + Math.PI);
            }
            double steerPower = Range.clip(-4.0 * offset / Math.PI, -1, 1);
            double drivePower = 1.25 * (wheelSpeed / WHEEL_CIRCUMFERENCE) * TICKS_PER_ROTATION / MAX_TICKS_PER_SECOND;
            if (i == 1 && !reversed || i == 0 && reversed) drivePower *= -1;
            motorPowers[2*i] = -steerPower - drivePower;
            motorPowers[2*i+1] = -steerPower + drivePower;
        }

        //Wheel powers must be in the range -1 to +1. If they are not, then we must scale down the values
        //of each wheel power, and also the values of vx, vy, and va
        double max = 1.0;
        for (int i = 0; i < 4; i++) max = Math.max(max, Math.abs(motorPowers[i]));

        for (int i=0; i<4; i++) motors[i].setPower(motorPowers[i]/max);

    }


    /**
     * Normalize an angle into the range of -PI to +PI radians
     * @param radians
     * @return
     */
    public static double normalizeRadians(double radians) {
        double temp = (radians + Math.PI) / (2.0 * Math.PI);
        return (temp - Math.floor(temp) - 0.5) * 2.0 * Math.PI;
    }



}
