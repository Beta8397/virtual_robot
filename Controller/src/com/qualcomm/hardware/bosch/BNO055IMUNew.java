package com.qualcomm.hardware.bosch;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import virtual_robot.controller.VirtualBot;

public class BNO055IMUNew implements IMU {

    private VirtualBot bot = null;
    private IMU.Parameters parameters = null;
    private double initialHeadingRadians = 0;
    private double headingRadians = 0;
    private double angularVelocityRadiansPerSec = 0;
    private boolean initialized = false;

    private long latencyNanos = 0;
    private long prevNanos = System.nanoTime();

    public BNO055IMUNew(VirtualBot bot){
        this.bot = bot;
    }

    public BNO055IMUNew(VirtualBot bot, int latencyMillis){
        this.bot = bot;
        latencyNanos = latencyMillis * 1000000;
    }

    /**
     * Initialize the BNO055IMU
     * @param parameters Parameters object
     * @return true to indicate initialization was successful
     */
    public synchronized boolean initialize(IMU.Parameters parameters){
        initialized = true;
        this.parameters = parameters;
        double tempHeadingRadians = bot.getHeadingRadians();
        headingRadians = tempHeadingRadians;
        initialHeadingRadians = tempHeadingRadians;
        prevNanos = System.nanoTime();
        return true;
    }

    public synchronized IMU.Parameters getParameters() { return parameters; }

    /**
     * Close the BNO055IMU
     */
    public synchronized void close(){
        angularVelocityRadiansPerSec = 0;
    }


    /**
     * Get the angular orientation (as an Orientation object), using the AxesReference, AxesOrder, and AngleUnit
     * specified by the arguments
     * @param reference axes reference
     * @param order axes order
     * @param angleUnit angle unit
     * @return angular orientation
     */
    public synchronized Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        if (!initialized) return null;

        double heading = headingRadians - initialHeadingRadians;
        if (heading > Math.PI) heading -= 2.0 * Math.PI;
        else if (heading < -Math.PI) heading += 2.0 * Math.PI;

        double piOver2;
        double firstAngle = 0.0, secondAngle = 0.0, thirdAngle = 0.0;
        if (angleUnit == org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES) {
            heading *= 180.0 / Math.PI;
            piOver2 = Math.PI / 2.0;
        } else {
            piOver2 = 90.0;
        }

        switch (order) {
            case ZXY: case ZXZ: case ZYX: case ZYZ:
                firstAngle = heading;
                break;
            case XZX: case XZY: case YZX: case YZY:
                secondAngle = heading;
                break;
            case XYZ: case YXZ:
                thirdAngle = heading;
                break;
            case YXY:
                secondAngle = heading;
                if (reference == AxesReference.INTRINSIC){
                    firstAngle = -piOver2;
                    thirdAngle = piOver2;
                } else {
                    firstAngle = piOver2;
                    thirdAngle = -piOver2;
                }
                break;
            case XYX:
                secondAngle = heading;
                if (reference == AxesReference.INTRINSIC){
                    firstAngle = piOver2;
                    thirdAngle = -piOver2;
                } else {
                    firstAngle = -piOver2;
                    thirdAngle = piOver2;
                }
        }
        return new Orientation(reference, order, angleUnit, (float)firstAngle, (float)secondAngle, (float)thirdAngle,
                System.nanoTime());

    }

    public synchronized YawPitchRollAngles getRobotYawPitchRollAngles(){
        double heading = headingRadians - initialHeadingRadians;
        if (heading > Math.PI) heading -= 2.0 * Math.PI;
        else if (heading < -Math.PI) heading += 2.0 * Math.PI;
        return new YawPitchRollAngles(AngleUnit.RADIANS, heading, 0, 0, System.nanoTime());
    }

    public synchronized Quaternion getRobotOrientationAsQuaternion(){
        double heading = headingRadians - initialHeadingRadians;
        if (heading > Math.PI) heading -= 2.0 * Math.PI;
        else if (heading < -Math.PI) heading += 2.0 * Math.PI;
        return new Quaternion((float)Math.cos(heading/2.0), 0, 0, (float)Math.sin(heading/2.0), System.nanoTime());
    }

    public synchronized AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit){
        float velocity = angleUnit == AngleUnit.RADIANS? (float)angularVelocityRadiansPerSec :
                (float)Math.toDegrees(angularVelocityRadiansPerSec);
        return new AngularVelocity(AngleUnit.RADIANS, 0, 0,
                velocity, System.nanoTime());
    }

    public synchronized void resetYaw(){
        double tempHeadingRadians = bot.getHeadingRadians();
        headingRadians = tempHeadingRadians;
        initialHeadingRadians = tempHeadingRadians;
    }

    /**
     * For internal use only
     * @param heading
     */
    public synchronized void updateHeadingRadians( double heading ){
        if (!initialized) {
            initialized = true;
            double tempHeadingRadians = bot.getHeadingRadians();
            headingRadians = tempHeadingRadians;
            initialHeadingRadians = tempHeadingRadians;
            prevNanos = System.nanoTime();
        }
        long nanos = System.nanoTime();
        if (nanos < (prevNanos + latencyNanos)) return;
        double deltaHeadingRadians = heading - headingRadians;
        if (deltaHeadingRadians > Math.PI) deltaHeadingRadians -= 2.0 * Math.PI;
        else if (deltaHeadingRadians < -Math.PI) deltaHeadingRadians += 2.0 * Math.PI;
        double seconds = (nanos - prevNanos) * 1.0E-9;
        angularVelocityRadiansPerSec = deltaHeadingRadians / seconds;
        headingRadians = heading;
        prevNanos = nanos;
    }

}
