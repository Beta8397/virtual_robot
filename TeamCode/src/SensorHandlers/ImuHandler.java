package SensorHandlers;

import Misc.ImuCalibration;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Created by root on 8/18/17.
 */

/*
    A class to handle the IMU for us - it sets up the gyroscope in the REV modules for us
 */
public class ImuHandler extends Thread {
    private long updateDelay = 200;
    private long lastUpdateStart;
    private BNO055IMU imu;
    private Orientation angles = new Orientation();
    private volatile boolean shouldRun;
    private double orientationOffset;
    private double previousOrientation;
    private int turnCount;
    private final double ANGLE_THRESHOLD = 45;
    private HardwareMap map;

    public ImuHandler(String name, double robotOrientationOffset, HardwareMap h){

        initIMU(name, h);
        shouldRun = true;
        orientationOffset = robotOrientationOffset;
        previousOrientation = 0;
        turnCount = 0;
        new Thread(new Runnable(){
            public void run(){
                while(shouldRun) {
                    lastUpdateStart = System.currentTimeMillis();
                    try{
                        updateIMU();
                    } catch (Exception e){
                        shouldRun = false;
                        throw new RuntimeException(e);
                    }
                    long timeLeft = updateDelay - (System.currentTimeMillis() - lastUpdateStart);
                    if(timeLeft > 0) safetySleep(timeLeft);
                }
            }
        }).start();
    }

    public void setOrientationOffset(double offset){
        orientationOffset = offset + orientationOffset;
    }

    private void safetySleep(long time){
        long start = System.currentTimeMillis();
        while(System.currentTimeMillis() - start < time && shouldRun);
    }

    private void initIMU(String name, HardwareMap hardwareMap){
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        BNO055IMU.CalibrationData dat = ImuCalibration.getCalibrationData();
        parameters.calibrationData = dat;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, name);
        imu.initialize(parameters);
        safetySleep(10);
        ImuCalibration.logImuStatus(imu);

        updateIMU();
    }

    private void updateIMU() {
        try {
            Orientation angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if(angles2 != null) {
                if (angles2.firstAngle >= -180 && angles2.firstAngle <= -180 + ANGLE_THRESHOLD / 2.0 && previousOrientation >= 180 - ANGLE_THRESHOLD / 2.0)
                    turnCount++;
                else if (angles2.firstAngle >= 180 - ANGLE_THRESHOLD / 2.0 && previousOrientation >= -180 && previousOrientation <= -180 + ANGLE_THRESHOLD / 2.0)
                    turnCount--;
                previousOrientation = angles2.firstAngle;
                angles = angles2;
            }
        } catch (Exception e){
            stopIMU();
            throw new RuntimeException(e);

        }
    }

    public double getFirstAngle() {
        return angles.firstAngle;
    }

    public double getOrientationOffset(){
        return  orientationOffset;
    }

    /*
        returns the orientation of the robot, 0 to 359 degrees
     */
    public double getOrientation(){
        double angle = -(angles.firstAngle + 360 * turnCount); // Z angle is the robot's orientation angle
        angle += orientationOffset;
        return angle;
    }

    public double[] getAngles() {
        double[] newAngles = {getOrientation(), angles.secondAngle, angles.thirdAngle};
        return newAngles;
    }

    public void stopIMU() {
        shouldRun = false;
    }
}