package DriveEngine.Ultimate;

import Misc.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.util.HashMap;

import javax.xml.ws.handler.HandlerResolver;

import Autonomous.ConfigVariables;
import Autonomous.HeadingVector;
import Autonomous.Location;
import Autonomous.Rectangle;
import MotorControllers.JsonConfigReader;
import MotorControllers.MotorController;
import MotorControllers.PIDController;
import SensorHandlers.ImuHandler;
import SensorHandlers.LIDARSensor;
import Misc.ConfigFile;

/**
 * Created by Jeremy on 8/23/2017.
 */

/*
    The base class for every opmode --- it sets up our drive system and contains all it's functions
 */
public class UltimateNavigation2 extends Thread {

    public static final Rectangle NO_GO_ZONE = new Rectangle(0, 0, 144, 48);

    private static final double GOOD_DIST_READING_TOLERANCE = 72.0;

    public MotorController[] driveMotors = new MotorController[4];
    public static final int FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR = 0;
    public static final int FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR = 1;
    public static final int BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR = 2;
    public static final int BACK_LEFT_HOLONOMIC_DRIVE_MOTOR = 3;

    public static final double LOCATION_DISTANCE_TOLERANCE = 0.75, ROUGH_LOCATION_DISTANCE_TOLERANCE = 1.5, STOPPING_DISTANCE_FACTOR = 0.2;
    public static final double LIDAR_DISTANCE_TOLERANCE = 15.0;
    public static final long TIME_TOLERANCE = 250;
    public static final long DEFAULT_DELAY_MILLIS = 10;

    public static final double FORWARD = 0;
    public static final double BACK = 180;
    public static final double RIGHT = 90;
    public static final double LEFT = -90;

    private volatile long threadDelayMillis = 10;

    public volatile double robotHeading = 0;

    private volatile double [] lastMotorPositionsInInches = {0,0,0,0};

    public PIDController headingController, turnController, cameraTranslationYController, cameraTranslationXController, cameraOrientationController, xPositionController, yPositionController;
    private volatile Location myLocation;
    private volatile HeadingVector[] wheelVectors = new HeadingVector[4];
    private volatile HeadingVector robotMovementVector = new HeadingVector();
    public ImuHandler orientation;
    private double orientationOffset = 0;

    private volatile boolean shouldRun = true, loggingData = true, usingSensors = false;
    private volatile long startTime = System.nanoTime();
    private volatile HeadingVector IMUTravelVector = new HeadingVector();

    private volatile Location IMUDistance = new Location(0, 0);
    private LIDARSensor[] distanceSensors;
    public static final int LEFT_SENSOR = 0, BACK_SENSOR = 1, RIGHT_SENSOR = 2, DRIVE_BASE = 3, FRONT_SENSOR = 4;
    private HashMap<Integer, int[]>[] updateLocationInformation = new HashMap[4]; // structure: {direction, {xSensor, ySensor}}
    public static final int Q1 = 0, Q2 = 1, Q3 = 2, Q4 = 3, NORTH = 0, SOUTH = 180, EAST = 90, WEST = 270;
    public static final double HEADING_TOLERANCE = 2.5;

    private final double HEADING_THRESHOLD = 2;
    private final double WHEEL_BASE_RADIUS = 20;
    private final double FL_WHEEL_HEADING_OFFSET = 45;
    private final double FR_WHEEL_HEADING_OFFSET = 315;
    private final double BR_WHEEL_HEADING_OFFSET = 45;
    private final double BL_WHEEL_HEADING_OFFSET = 315;
    private double acceleration = 0;
    private HardwareMap hardwareMap;

    public UltimateNavigation2(HardwareMap hw, Location startLocation, double robotOrientationOffset, String configFile, boolean ignoreInitialSensorLocation) throws RuntimeException {
        hardwareMap = hw;
        initializeUsingConfigFile(configFile);
        populateHashmaps();
        orientationOffset = robotOrientationOffset;
        orientation = new ImuHandler("imu", orientationOffset, hardwareMap);
        myLocation = new Location(startLocation.getX(),startLocation.getY(), robotOrientationOffset);
        distanceSensors = new LIDARSensor[3];
        distanceSensors[LEFT_SENSOR] = new LIDARSensor(hardwareMap.get(DistanceSensor.class, "left"), LEFT_SENSOR, "left");
        distanceSensors[BACK_SENSOR] = new LIDARSensor(hardwareMap.get(DistanceSensor.class, "back"), BACK_SENSOR, "back");
        distanceSensors[RIGHT_SENSOR] = new LIDARSensor(hardwareMap.get(DistanceSensor.class, "right"), RIGHT_SENSOR, "right");
//        distanceSensors[LEFT_SENSOR].getDistance(); We shouldn't need these because of initSensor in LIDARSensor class
//        distanceSensors[BACK_SENSOR].getDistance();
//        distanceSensors[RIGHT_SENSOR].getDistance();
//        if (!ignoreInitialSensorLocation) getInitialLocation();

        for (int i = 0; i < wheelVectors.length; i++)
            wheelVectors[i] = new HeadingVector();

        for (int i = 0; i < lastMotorPositionsInInches.length; i ++)
            lastMotorPositionsInInches[i] = driveMotors[i].getInchesFromStart();

        robotMovementVector = new HeadingVector();
        startTime = System.nanoTime();
        new Thread(new Runnable() {
            @Override
            public void run() {
                for (int i = 0; i < driveMotors.length; i++) {
                    Log.d("Inch from start", Integer.toString(i) + ": " + driveMotors[i].getInchesFromStart());
                }
                while (shouldRun) {
                    try {
                        if (loggingData) updateData();
                    }
                    catch (Exception e) {
                        shouldRun = false;
                        throw new RuntimeException(e);
                    }
                    safetySleep(threadDelayMillis);
                }
            }
        }).start();
    }

    public UltimateNavigation2(HardwareMap hw, Location startLocation, double robotOrientationOffset, String configFile) throws RuntimeException {
        this(hw, startLocation, robotOrientationOffset, configFile, false);
    }

    public UltimateNavigation2(HardwareMap hw, Location startLocation, String configFileLoc) throws RuntimeException {
        this(hw, startLocation, startLocation.getHeading(), configFileLoc);
    }

    private void getInitialLocation() {
        // SOME LOGICAL ERROR IN FINDING QUADRANT MOST LIKELY -- CHECK HASHTABLE FOR LOCATION CHECKS
        int quadrant = -1;
        if(myLocation.getX() >= 0 && myLocation.getY() >= 0) quadrant = Q1;
        else if(myLocation.getX() < 0 && myLocation.getY() >= 0) quadrant = Q2;
        else if(myLocation.getX() < 0 && myLocation.getY() < 0) quadrant = Q3;
        else if(myLocation.getX() >= 0 && myLocation.getY() < 0) quadrant = Q4;
        Log.d("Quadrant: ", ""+quadrant);

        double simpleHeading = myLocation.getHeading() % 360;
        int dir = -1;
        if(simpleHeading < 1 && simpleHeading > -1) dir = NORTH;
        else if(simpleHeading < 91 && simpleHeading > 89) dir = EAST; // REVIEW: suggest to define HEADING_TOLERANCE = 1 and use abs(simpleHeading - 90) < HEADING_TOLERANCE
        else if(simpleHeading < 181 && simpleHeading > 179) dir = SOUTH;
        else if(simpleHeading < 271 && simpleHeading > 269) dir = WEST;
        Log.d("Direction: ", ""+dir);

        int[] sensorsToUse = updateLocationInformation[quadrant].get(dir);
        if (sensorsToUse != null && sensorsToUse[0] != DRIVE_BASE) {
            double x = 71.0 - distanceSensors[sensorsToUse[0]].getDistance() - 7.0;
            x *= quadrant == Q2 || quadrant == Q3 ? -1 : 1; //TODO this is wrong I'm pretty sure
            Log.d("Start sensor X: ", "" + x);
            Log.d("Start loc X: ", myLocation.getX()+"");
            if (Math.abs(myLocation.getX() - x) < 50) myLocation.setX(x);
        }
        if (sensorsToUse != null && sensorsToUse[1] != DRIVE_BASE) {
            double y = 71.0 - distanceSensors[sensorsToUse[1]].getDistance() - 7.0;
            y *= quadrant == Q3 || quadrant == Q4 ? -1 : 1;
            Log.d("Start sensor Y: ", "" + y);
            Log.d("Start loc Y:", myLocation.getY()+"");
            if (Math.abs(myLocation.getY() - y) < 50) myLocation.setY(y);
        }
        // TODO how can we stop both run and logcat from filling up with IMU Locations? It's really annoying cuz i can't see what's printed out from AnnieNavigation -- type in your filter, right now its location...
    }

    public void stopLoggingData() {
        loggingData = false;
    }

    public void startLoggingData() {
        loggingData = true;
    }

    public void useSensorLocationTracking() {
        usingSensors = true;
    }

    public void disableSensorLocationTracking() {
        usingSensors = false;
    }

    public void setOrientationOffset(double offset) {
        orientationOffset = offset;
        orientation.setOrientationOffset(orientationOffset);
    }

    private void updateLastMotorPositionsInInches() {
        for (int i = 0; i < driveMotors.length; i++){
            lastMotorPositionsInInches[i] = driveMotors[i].getInchesFromStart();
        }
    }

    private void updateHeading() {
        robotHeading = (orientation.getOrientation());
    }

    public Location getRobotLocation() {
        return new Location(myLocation.getX(), myLocation.getY());
    }

    private void updateLocation() {
        boolean shouldTranslateX = false, shouldTranslateY = false;
        double expectedY, expectedX;
        double simpleHeading = robotHeading % 360;

        // sensor location tracking
        int quadrant = -1;
        if(myLocation.getX() >= 0 && myLocation.getY() >= 0) quadrant = Q1;
        else if(myLocation.getX() < 0 && myLocation.getY() >= 0) quadrant = Q2;
        else if(myLocation.getX() < 0 && myLocation.getY() < 0) quadrant = Q3;
        else if(myLocation.getX() >= 0 && myLocation.getY() < 0) quadrant = Q4;
        int dir = -1;
        if(Math.abs(simpleHeading - NORTH) < HEADING_TOLERANCE) dir = NORTH;
        else if(Math.abs(simpleHeading - EAST) < HEADING_TOLERANCE) dir = EAST;
        else if (Math.abs(simpleHeading - SOUTH) < HEADING_TOLERANCE) dir = SOUTH;
        else if (Math.abs(simpleHeading - WEST) < HEADING_TOLERANCE) dir = WEST;

//        if(simpleHeading < 2.5 && simpleHeading > -2.5) dir = NORTH;
//        else if(simpleHeading < 92.5 && simpleHeading > 87.5) dir = EAST; // REVIEW: suggest to define HEADING_TOLERANCE = 1 and use abs(simpleHeading - 90) < HEADING_TOLERANCE
//        else if(simpleHeading < 182.5 && simpleHeading > 177.5) dir = SOUTH;
//        else if(simpleHeading < 272.5 && simpleHeading > 267.5) dir = WEST;
        else {
            // if not lined up to a square direction, then just use wheel odometry to track position
            shouldTranslateX = true;
            shouldTranslateY = true;
        }

        // wheel location tracking
        HeadingVector travelVector = wheelVectors[0].addVectors(wheelVectors);
        travelVector = new HeadingVector(travelVector.x() / 2, travelVector.y() / 2);
        double headingOfRobot = travelVector.getHeading();
        double magnitudeOfRobot = travelVector.getMagnitude();
        double actualHeading = (headingOfRobot + robotHeading) % 360;
        robotMovementVector.calculateVector(actualHeading, magnitudeOfRobot);
        double deltaX = robotMovementVector.x();
        double deltaY = robotMovementVector.y();

        if (!myLocation.withinRectangle(NO_GO_ZONE) && usingSensors) {
            if (!shouldTranslateX) {
                int[] sensorsToUse = updateLocationInformation[quadrant].get(dir);
                if (sensorsToUse != null && sensorsToUse[0] == DRIVE_BASE) shouldTranslateX = true;
                else if (sensorsToUse != null && sensorsToUse[1] == DRIVE_BASE)
                    shouldTranslateY = true;
                if (!shouldTranslateX && 
                        (myLocation.withinRectangle(ConfigVariables.VALID_X_SENSOR_READ_AREA_1_RED) || myLocation.withinRectangle(ConfigVariables.VALID_X_SENSOR_READ_AREA_2_RED)
                        || myLocation.withinRectangle(ConfigVariables.VALID_X_SENSOR_READ_AREA_1_BLUE) || myLocation.withinRectangle(ConfigVariables.VALID_X_SENSOR_READ_AREA_2_BLUE))) {
                    // REVIEW: there are several conditions to check to determine if a distance reading is good, so
                    // is is best to encapsulate this in a class.  I suggest to have a sensor.getGoodDistance()
                    // function that returns a Double distance or null, if a good distances isn't available.
                    // Then, have a separate function that can be called in the null case, for example sensor.explainNull()
                    // to get a message describing the problem, which can be reported in telemetry or logs.
                    // An alternate implementation would have getDistance() return a double only if a good distance is
                    // available and throw InvalidDistanceException otherwise.  Then, this code can catch the exception
                    // to report in telemetry or logs.
                    Double dist = distanceSensors[sensorsToUse[0]].getGoodDistance();
                    Log.d("X sensor dist: ", dist+"");
                    if (dist != null && dist < GOOD_DIST_READING_TOLERANCE) {
                        // REVIEW: the magic numbers in the following formula should be pulled out as named constants
                        expectedX = 71.0 - dist - 7.0;
                        if (quadrant == Q2 || quadrant == Q3) expectedX *= -1;
                        if (Math.abs(expectedX - (myLocation.getX() + deltaX)) <= LIDAR_DISTANCE_TOLERANCE)
                            myLocation.setX(expectedX);
                        else shouldTranslateX = true;
                    } else {
                        shouldTranslateX = true;
                    }
                } else shouldTranslateX = true;
                if (!shouldTranslateY &&
                        (myLocation.withinRectangle(ConfigVariables.VALID_Y_SENSOR_READ_AREA_1_RED) || myLocation.withinRectangle(ConfigVariables.VALID_Y_SENSOR_READ_AREA_2_RED)
                        || myLocation.withinRectangle(ConfigVariables.VALID_Y_SENSOR_READ_AREA_1_BLUE) || myLocation.withinRectangle(ConfigVariables.VALID_Y_SENSOR_READ_AREA_2_BLUE))) {
                    Double dist = distanceSensors[sensorsToUse[1]].getGoodDistance();
                    if (dist != null && dist < GOOD_DIST_READING_TOLERANCE) {
                        expectedY = 71.0 - dist - 7.0;
                        if (quadrant == Q3 || quadrant == Q4) expectedY *= -1;
                        myLocation.setY(expectedY);
                        if (Math.abs(expectedY - (myLocation.getY() + deltaY)) <= LIDAR_DISTANCE_TOLERANCE)
                            myLocation.setY(expectedY);
                        else shouldTranslateY = true;
                    } else {
                        shouldTranslateY = true;
                    }
                } else shouldTranslateY = true;
            }
        }else{
            shouldTranslateX = shouldTranslateY = true;
        }
        if(shouldTranslateX) myLocation.addX(deltaX);
        if(shouldTranslateY) myLocation.addY(deltaY);
        myLocation.setHeading(restrictAngle(orientation.getOrientation(), 0));
        Log.d("Location", "X:" + myLocation.getX() + " Y:" + myLocation.getY());
        Log.d("Sensor X:", shouldTranslateX ? "ENCODER" : "LIDAR");
        Log.d("Sensor Y:", shouldTranslateY ? "ENCODER" : "LIDAR");
    }

    private double restrictAngle(double angleToChange, double referenceAngle) {
        while(angleToChange < referenceAngle - 180) angleToChange += 360;
        while (angleToChange > referenceAngle + 180) angleToChange -= 360;
        return angleToChange;
    }

    private void updateIMUTrackedDistance() {
//        double deltaTime = (System.nanoTime() - startTime) * (1.0/1.0e9);
//
//        HeadingVector travelVector = new HeadingVector(orientation.getAccelerations()[0], orientation.getAccelerations()[1]);
//        double accelerationHeading = travelVector.getHeading();
//        double accelerationMagnitude = travelVector.getMagnitude();
//        double actualHeading = (accelerationHeading + robotHeading)%360;
//        IMUTravelVector.calculateVector(actualHeading, accelerationMagnitude);
//        double deltaX = 0.5*IMUTravelVector.x()*deltaTime;
//        double deltaY = 0.5*IMUTravelVector.y()*deltaTime;
//        IMUDistance.addXY(deltaX, deltaY);
//
//        Log.d("IMU Distance: ", "" + IMUDistance.distanceToLocation(new Location(0, 0)));
//        Log.d("IMU Location: ", IMUDistance.toString());
//
//        startTime = System.nanoTime();
    }

    public void setLocation(Location loc) {
        myLocation = new Location(loc.getX(), loc.getY());
    }

    private void updateData(){

        updateHeading();
        wheelVectors = getWheelVectors();
        updateLocation();
//        updateIMUTrackedDistance();
    }

    private void safetySleep(long time){
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < time && shouldRun);
    }

    public void setThreadDelayMillis(long delayMillis){
        threadDelayMillis = delayMillis;
    }

    public double getOrientation(){
        return robotHeading;
    }

    public void initializeUsingConfigFile(String file) {
        InputStream stream = null;
        try {
            stream = ConfigFile.open(hardwareMap, file);
        }
        catch(Exception e) {
            Log.d("Drive Engine Error: ",e.toString());
            throw new RuntimeException("Drive Engine Open Config File Fail: " + e.toString());
        }
        JsonConfigReader reader = new JsonConfigReader(stream);
        try {
            driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = new MotorController(reader.getString("FRONT_LEFT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = new MotorController(reader.getString("FRONT_RIGHT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = new MotorController(reader.getString("BACK_LEFT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = new MotorController(reader.getString("BACK_RIGHT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            for (int i = 0; i < driveMotors.length; i++) {
                driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(reader.getString("DRIVE_MOTOR_BRAKING_MODE").equals("BRAKE")){
                for (int i = 0; i < driveMotors.length; i++) {
                    driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
            else if(reader.getString("DRIVE_MOTOR_BRAKING_MODE").equals("FLOAT")){
                for (int i = 0; i < driveMotors.length; i++) {
                    driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }
            if(reader.getString("FRONT_LEFT_MOTOR_DIRECTION").equals("REVERSE")) {
                driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else if(reader.getString("FRONT_LEFT_MOTOR_DIRECTION").equals("FORWARD")) {
                driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if(reader.getString("FRONT_RIGHT_MOTOR_DIRECTION").equals("REVERSE")) {
                driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else if(reader.getString("FRONT_RIGHT_MOTOR_DIRECTION").equals("FORWARD")) {
                driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if(reader.getString("BACK_RIGHT_MOTOR_DIRECTION").equals("REVERSE")) {
                driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else if(reader.getString("BACK_RIGHT_MOTOR_DIRECTION").equals("FORWARD")) {
                driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if (reader.getString("BACK_LEFT_MOTOR_DIRECTION").equals("REVERSE")) {
                driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else if (reader.getString("BACK_LEFT_MOTOR_DIRECTION").equals("FORWARD")) {
                driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
            }
            headingController = new PIDController(reader.getDouble("HEADING_Kp"), reader.getDouble("HEADING_Ki"), reader.getDouble("HEADING_Kd"));
            headingController.setIMax(reader.getDouble("HEADING_Ki_MAX"));
            turnController = new PIDController(reader.getDouble("TURN_Kp"), reader.getDouble("TURN_Ki"), reader.getDouble("TURN_Kd"));
            turnController.setIMax(reader.getDouble("TURN_Ki_MAX"));
            cameraTranslationYController = new PIDController(reader.getDouble("CAMERA_TRANSLATION_Y_Kp"), reader.getDouble("CAMERA_TRANSLATION_Y_Ki"), reader.getDouble("CAMERA_TRANSLATION_Y_Kd"));
            cameraTranslationYController.setIMax(reader.getDouble("CAMERA_TRANSLATION_Y_Ki_MAX"));
            cameraTranslationXController = new PIDController(reader.getDouble("CAMERA_TRANSLATION_X_Kp"), reader.getDouble("CAMERA_TRANSLATION_X_Ki"), reader.getDouble("CAMERA_TRANSLATION_X_Kd"));
            cameraTranslationXController.setIMax(reader.getDouble("CAMERA_TRANSLATION_X_Ki_MAX"));
            cameraOrientationController = new PIDController(reader.getDouble("CAMERA_ORIENTATION_Kp"), reader.getDouble("CAMERA_ORIENTATION_Ki"), reader.getDouble("CAMERA_ORIENTATION_Kd"));
            cameraOrientationController.setIMax(reader.getDouble("CAMERA_ORIENTATION_Ki_MAX"));
            xPositionController = new PIDController(reader.getDouble("X_POSITION_Kp"), reader.getDouble("X_POSITION_Ki"), reader.getDouble("X_POSITION_Kd"));
            yPositionController = new PIDController(reader.getDouble("Y_POSITION_Kp"), reader.getDouble("Y_POSITION_Ki"), reader.getDouble("Y_POSITION_Kd"));
            xPositionController.setIMax(reader.getDouble("X_POSITION_Ki_MAX"));
            yPositionController.setIMax(reader.getDouble("Y_POSITION_Ki_MAX"));
        } catch(Exception e){
            Log.e(" Drive Engine Error", "Config File Read Fail: " + e.toString());
            throw new RuntimeException("Drive Engine Config Read Failed!:" + e.toString());
        }
    }

    public void correctedDriveOnHeadingIMU(double heading, double desiredVelocity, LinearOpMode mode) {
        correctedDriveOnHeadingIMU(heading,desiredVelocity,DEFAULT_DELAY_MILLIS,mode);
    }

    public void correctedDriveOnHeadingIMU(double heading, double desiredVelocity, long delayTimeMillis, LinearOpMode mode) {
        desiredVelocity = Math.abs(desiredVelocity);
        double curOrientation = orientation.getOrientation();
        double distanceFromHeading = 0;
        distanceFromHeading = heading - curOrientation; // as in -1 if heading is 0 and current orientation is 1
        if(distanceFromHeading > 180) distanceFromHeading -= 360;
        if(distanceFromHeading < -180) distanceFromHeading += 360;
        if(curOrientation > 315 || curOrientation <= 45){
            headingController.setSp(0);
        }
        else if(curOrientation > 45 && curOrientation <= 135){
            headingController.setSp(90);
        }
        else if(curOrientation > 135 && curOrientation <= 225){
            headingController.setSp(180);
        }
        else if(curOrientation > 225 && curOrientation <= 315){
            headingController.setSp(270);
        }
        double distanceFromSetPoint = headingController.getSp() - curOrientation;
        if(distanceFromSetPoint < -180) distanceFromSetPoint += 360;
        else if(distanceFromSetPoint > 180) distanceFromSetPoint -= 360;
        double deltaVelocity = headingController.calculatePID(distanceFromSetPoint + headingController.getSp()); //isue with this line...
        if(distanceFromHeading < 0) distanceFromHeading += 360;
        else if(distanceFromHeading > 360) distanceFromHeading -= 360;
        double [] velocities = determineMotorVelocitiesToDriveOnHeading(heading - curOrientation,desiredVelocity);

        if(distanceFromHeading > 315 || distanceFromHeading <= 45){
            velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
            velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
            velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
            velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
        }

        else if(distanceFromHeading > 45 && distanceFromHeading <= 135){
            velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
            velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
            velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
            velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
        }

        else if(distanceFromHeading > 135 && distanceFromHeading <= 225){
            velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
            velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
            velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
            velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
        }

        else if(distanceFromHeading > 225 && distanceFromHeading <= 315){
            velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
            velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
            velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
            velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
        }

        applyMotorVelocities(velocities);
        mode.sleep(delayTimeMillis);
    }

    public void correctedDriveOnHeadingIMURotation(double heading, double desiredVelocity, long delayTimeMillis, LinearOpMode mode) {
        desiredVelocity = Math.abs(desiredVelocity);
        double curOrientation = orientation.getOrientation();
        double turnCorrection = turnController.calculatePID(curOrientation);
        if(Double.isNaN(turnCorrection)) turnCorrection = 0;
        Log.d("Delta Velocity:", "" + turnCorrection);
        double [] velocities = determineMotorVelocitiesToDriveOnHeading(heading/* - curOrientation*/, desiredVelocity);

        Log.d("Initial Velocities:", "-----------");
        for(int i = 0; i < velocities.length; i++) {
            Log.d("Motor " + " Velocity", "" + velocities[i]);
        }

        double [] rotationalCorrections = calculateTurnVelocities(turnCorrection);
        for(int i = 0; i < driveMotors.length; i++) {
            velocities[i] += rotationalCorrections[i];
        }

        Log.d("Final Velocities:", "-----------");
        for(int i = 0; i < velocities.length; i++) {
            Log.d("Motor " + " Velocity", "" + velocities[i]);
        }

        applyMotorVelocities(velocities);
        mode.sleep(delayTimeMillis);
    }

    public void driveOnHeadingPID(double heading, double desiredVelocity, LinearOpMode mode) {
        driveOnHeadingPID(heading, desiredVelocity, DEFAULT_DELAY_MILLIS, mode);
    }

    // Note: set turn Sp prior to calling
    public void driveOnHeadingPID(double heading, double desiredVelocity, long delayTimeMillis, LinearOpMode mode) {
        desiredVelocity = Math.abs(desiredVelocity);
        double curOrientation = orientation.getOrientation();
        double turnCorrection = turnController.calculatePID(curOrientation/*distanceFromSetPoint + headingController.getSp()*/); //issue with this line...
        if (Double.isNaN(turnCorrection)) turnCorrection = 0;
        Log.d("Delta Velocity:", "" + turnCorrection);
        double [] velocities = determineMotorVelocitiesToDriveOnHeading(heading, desiredVelocity);

        Log.d("Initial Velocities:", "-----------");
        for(int i = 0; i < velocities.length; i++) {
            Log.d("Motor " + " Velocity", "" + velocities[i]);
        }

        double [] rotationalCorrections = calculateTurnVelocities(turnCorrection);
        for(int i = 0; i < driveMotors.length; i++) {
            velocities[i] += rotationalCorrections[i];
        }
//        if(heading > 315 || heading <= 45){
//            Log.d("Wheels Affected", "-Left+Right");
//            velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity; // 0
//            velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity; // 1
//            velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity; // 3
//            velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity; // 2
//        }
//
//        else if(heading > 45 && heading <= 135){
//            Log.d("Wheels Affected", "-Front+Back");
//            velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity; // 0
//            velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity; // 1
//            velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity; // 3
//            velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity; // 2
//        }
//
//        else if(heading > 135 && heading <= 225){
//            Log.d("Wheels Affected", "-Right+Left");
//            velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
//            velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
//            velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
//            velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
//        }
//
//        else if(heading > 225 && heading <= 315){
//            Log.d("Wheels Affected", "-Back+Front");
//            velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
//            velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
//            velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
//            velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
//        }

        Log.d("Final Velocities:", "-----------");
        for(int i = 0; i < velocities.length; i++) {
            Log.d("Motor " + " Velocity", "" + velocities[i]);
        }

        applyMotorVelocities(velocities);
        mode.sleep(delayTimeMillis);
    }

    public void driveDistanceToLocation(Location target, double desiredVelocity, LinearOpMode mode) {
        double heading = Math.toDegrees(Math.atan2(target.getY() - myLocation.getY(), target.getX() - myLocation.getX())) - 90;
        heading = (360 - heading) - orientationOffset;
        heading %= 360;
        if(heading >= 360) heading -= 360;
        if(heading < 0) heading += 360;
        Log.d("Drive Distance Heading", ""+heading);
        driveDistance(myLocation.distanceToLocation(target), heading, desiredVelocity, mode);
    }

    public void driveDistance(double distanceInInches, double heading, double desiredVelocity, LinearOpMode mode) {
//        for(MotorController m : driveMotors) {
//            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//        mode.idle();
//        for(MotorController m : driveMotors) {
//            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        mode.idle();

//        updateLastMotorPositionsInInches();

        distanceInInches = Math.abs(distanceInInches);
        double distanceTraveled = 0;
        double [] motorPositionsInches = getMotorPositionsInches();
        double [] startPositionsInches = motorPositionsInches;
        double [] deltaInches;
        double averagePosition = 0;
        if (heading >= 360) heading -= 360;
        else if (heading < 0) heading += 360;
        double curOrientation = orientation.getOrientation();
        turnController.setSp(curOrientation);
        while (distanceTraveled < distanceInInches && mode.opModeIsActive()) {
            //from our motor position, determine location
            driveOnHeadingPID(heading,desiredVelocity,0, mode);
            motorPositionsInches = getMotorPositionsInches();
            deltaInches = new double[4];
            averagePosition = 0;
            if(heading == 45 || heading == 225) {
                deltaInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]);
                deltaInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
                averagePosition += deltaInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] + deltaInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR];
                averagePosition /= 2.0;
                distanceTraveled = averagePosition;
            } else if(heading == 135 || heading == 315) {
                deltaInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
                deltaInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]);
                averagePosition += deltaInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] + deltaInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR];
                averagePosition /= 2.0;
                distanceTraveled = averagePosition;
            } else {
                for (int i = 0; i < motorPositionsInches.length; i++) {
                    deltaInches[i] = Math.abs(motorPositionsInches[i] - startPositionsInches[i]);
//                    mode.telemetry.addData("Delta: ", motorPositionsInches[i] - startPositionsInches[i]);
                }
                mode.telemetry.update();
                for (double i : deltaInches) {
                    averagePosition += i;
                }
                averagePosition /= (double) deltaInches.length;
                distanceTraveled = averagePosition / Math.sin(Math.toRadians(45.0));
            }
            Log.d("Distance Travelled", "" + distanceTraveled);
        }
        brake();
        Log.d("Location", getRobotLocation().toString());
    }

    public void driveDistanceAccelerationBased(double distanceInInches, double heading, double desiredVelocity, LinearOpMode mode) {
        for(MotorController m : driveMotors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        mode.idle();
        for(MotorController m : driveMotors) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        distanceInInches = Math.abs(distanceInInches);
        double distanceTraveled = 0;
        double [] motorPositionsInches = getMotorPositionsInches();
        double [] startPositionsInches = motorPositionsInches;
        double [] deltaInches;
        double averagePosition = 0;
        double currVelocity = 0;
        double accel = Math.pow(desiredVelocity, 2)/(2*distanceInInches);
        if (heading >= 360) heading -= 360;
        else if (heading < 0) heading += 360;
        double curOrientation = orientation.getOrientation();
        turnController.setSp(curOrientation);
        double endTime = distanceInInches / desiredVelocity;
        double startTime = System.currentTimeMillis() / 1000.0;
        while (distanceTraveled < distanceInInches && mode.opModeIsActive()) {
            double deltaTime = (System.currentTimeMillis() / 1000.0) - startTime;
            //ACCELERATION CODE
            if (deltaTime <= 0.5) {
                currVelocity = desiredVelocity * deltaTime / 0.5;
            } else if (distanceTraveled >= (2.0/3.0 * distanceInInches)) {
                currVelocity = desiredVelocity * (1 - (distanceTraveled - (2.0/3.0) * distanceInInches) / (1.0/3.0 * distanceInInches));
            } else {
                currVelocity = desiredVelocity;
            }
            //from our motor position, determine location
            driveOnHeadingPID(heading,currVelocity,0, mode);
            motorPositionsInches = getMotorPositionsInches();
            deltaInches = new double[4];
            averagePosition = 0;
            if(heading == 45 || heading == 225) {
                deltaInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]);
                deltaInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
                averagePosition += deltaInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] + deltaInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR];
                averagePosition /= 2.0;
                distanceTraveled = averagePosition;
            } else if(heading == 135 || heading == 315) {
                deltaInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
                deltaInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]);
                averagePosition += deltaInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] + deltaInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR];
                averagePosition /= 2.0;
                distanceTraveled = averagePosition;
            } else {
                for (int i = 0; i < motorPositionsInches.length; i++) {
                    deltaInches[i] = Math.abs(motorPositionsInches[i] - startPositionsInches[i]);
//                    mode.telemetry.addData("Delta: ", motorPositionsInches[i] - startPositionsInches[i]);
                }
                mode.telemetry.update();
                for (double i : deltaInches) {
                    averagePosition += i;
                }
                averagePosition /= (double) deltaInches.length;
                distanceTraveled = averagePosition / Math.sin(Math.toRadians(45.0));
            }
            Log.d("Distance Travelled", "" + distanceTraveled);
        }
        brake();
        Log.d("Location", getRobotLocation().toString());
    }

    public void driveDistanceNonCorrected(double distanceInInches, double heading, double desiredVelocity, LinearOpMode mode) {
        distanceInInches = Math.abs(distanceInInches);
        double distanceTraveled = 0;
        double [] motorPositionsInches = getMotorPositionsInches();
        double [] startPositionsInches = motorPositionsInches;
        double [] deltaInches;
        double averagePosition = 0;
        while(distanceTraveled < distanceInInches && mode.opModeIsActive()){
            //from our motor position, determine location
            driveOnHeading((int)(heading + .5),desiredVelocity);
            motorPositionsInches = getMotorPositionsInches();
            deltaInches = new double[4];
            averagePosition = 0;
            if(heading == 45 || heading == 225){
                deltaInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]);
                deltaInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
                averagePosition += deltaInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] + deltaInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR];
                averagePosition /= 2;
                distanceTraveled = averagePosition;
            } else if(heading == 135 || heading == 315){
                deltaInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
                deltaInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]);
                averagePosition += deltaInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] + deltaInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR];
                averagePosition /= 2;
                distanceTraveled = averagePosition;
            } else {
                for (int i = 0; i < motorPositionsInches.length; i++) {
                    deltaInches[i] = Math.abs(motorPositionsInches[i] - startPositionsInches[i]);
                }
                for (double i : deltaInches) {
                    averagePosition += i;
                }
                averagePosition /= (double) deltaInches.length;
                distanceTraveled = averagePosition / Math.sin(Math.toRadians(45));
            }
        }
        brake();
    }

    long [] getMotorPositionsTicks(){
        long [] positions = new long[4];
        positions[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        positions[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        positions[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        positions[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        return  positions;
    }

    public double [] getMotorPositionsInches(){
        double [] inches = new double [4];
        long [] ticks = getMotorPositionsTicks();
        inches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].convertTicksToInches(ticks[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]));
        inches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].convertTicksToInches(ticks[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]));
        inches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].convertTicksToInches(ticks[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]));
        inches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].convertTicksToInches(ticks[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]));
        return inches;
    }

    double [] determineMotorVelocitiesToDriveOnHeading(double heading, double desiredVelocity) {
        double[] velocities = new double[4];
        velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredVelocity * Math.sin(Math.toRadians(heading + 45));
        velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = desiredVelocity * Math.cos(Math.toRadians(heading + 45));
        velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = desiredVelocity * Math.sin(Math.toRadians(heading + 45));
        velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredVelocity * Math.cos(Math.toRadians(heading + 45));
        return velocities;
    }

    public void driveOnHeading(double heading, double desiredVelocity) {
        applyMotorVelocities(determineMotorVelocitiesToDriveOnHeading(heading, desiredVelocity));
    }

    public void driveOnHeadingWithTurning(double heading, double movementPower, double turnPower){
        double [] movementPowers = calculatePowersToDriveOnHeading(heading, movementPower);
        double [] turningPowers = calculatePowersToTurn(turnPower);
        double [] total = new double[4];
        for (int i = 0; i < movementPowers.length; i ++) {
            total[i] = movementPowers[i] + turningPowers[i];
        }
        normalizePowers(total);
        applyMotorPowers(total);
    }

    private double [] calculatePowersToDriveOnHeading(double heading, double desiredPower){
        double[] powers = new double[4];
        if(desiredPower == 0){
            for(int i = 0; i < powers.length; i ++){
                powers[i] = 0;
            }
            return powers;
        }
        powers[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredPower * Math.sin(Math.toRadians(heading + 45));
        powers[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = desiredPower * Math.cos(Math.toRadians(heading + 45));
        powers[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = desiredPower * Math.sin(Math.toRadians(heading + 45));
        powers[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredPower * Math.cos(Math.toRadians(heading + 45));
        //Log.d("MotorPow","" + powers[0]);
        return powers;
    }

    private double[] calculatePowersToTurn(double desiredTurnRateOfMax) {
        double[] powers = new double[4];
        if (desiredTurnRateOfMax == 0) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] = 0;
            }
            return powers;
        }
        powers[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredTurnRateOfMax;
        powers[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -desiredTurnRateOfMax;
        powers[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredTurnRateOfMax;
        powers[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -desiredTurnRateOfMax;
        for (int i = 0; i < powers.length; i++) {
            if (Double.isNaN(powers[i])) powers[i] = 0;
        }
        return powers;
    }

    private void normalizePowers(double[] toNormalize) {
        //get the min and max powers
        double min = toNormalize[0], max = toNormalize[0];
        for (int i = 0; i < toNormalize.length; i++) {
            if (toNormalize[i] < min) min = toNormalize[i];
            else if (toNormalize[i] > max) max = toNormalize[i];
        }
        //assign toScaleAgainst to the largest (abs) value
        double toScaleAgainst = 0;
        if (Math.abs(min) < Math.abs(max)) toScaleAgainst = Math.abs(max);
        else toScaleAgainst = Math.abs(min);
        //if the largest (abs) is greater than 1, scale all values appropriately
        if(toScaleAgainst > 1){
            for(int i = 0; i < toNormalize.length; i ++){
                toNormalize[i] = toNormalize[i] / toScaleAgainst;
            }
        }
    }

    public void relativeDriveOnHeadingWithTurning(double heading, double driveVelocity, double magnitudeOfTurn) {
        driveVelocity = Math.abs(driveVelocity);
        double curOrientation = orientation.getOrientation();
        double distanceFromHeading = 0;
        distanceFromHeading = heading - curOrientation; // as in -1 if heading is 0 and current orientation is 1
        if(distanceFromHeading > 180) distanceFromHeading -= 360;
        if(distanceFromHeading < -180) distanceFromHeading += 360;
        if(curOrientation > 315 || curOrientation <= 45){
            headingController.setSp(0);
        }
        else if(curOrientation > 45 && curOrientation <= 135){
            headingController.setSp(90);
        }
        else if(curOrientation > 135 && curOrientation <= 225){
            headingController.setSp(180);
        }
        else if(curOrientation > 225 && curOrientation <= 315){
            headingController.setSp(270);
        }
        double distanceFromSetPoint = headingController.getSp() - curOrientation;
        if(distanceFromSetPoint < -180) distanceFromSetPoint += 360;
        else if(distanceFromSetPoint > 180) distanceFromSetPoint -= 360;
        if(distanceFromHeading < 0) distanceFromHeading += 360;
        else if(distanceFromHeading > 360) distanceFromHeading -= 360;
        double [] headingVelocities = determineMotorVelocitiesToDriveOnHeading(heading - curOrientation,driveVelocity);
        //real quick, make distance from heading always positive
        double [] turnVelocities =  calculateTurnVelocitiesRelativeToMax(magnitudeOfTurn);
        double [] finalVelocities = new double[4];
        for(int i = 0; i < finalVelocities.length; i ++){
            finalVelocities[i] = turnVelocities[i] + headingVelocities[i];
        }
        //look for max and min values
        double maxValue = finalVelocities[0];
        double minValue = finalVelocities[0];
        for(int i = 1; i < finalVelocities.length; i ++){
            if(finalVelocities[i] > maxValue){
                maxValue = finalVelocities[i];
            }
            else if(finalVelocities[i] < minValue){
                minValue = finalVelocities[i];
            }
        }
        //scale all motor powers to correspond withf maxVelocities
        double scaleValue = 1;
        if(Math.abs(maxValue) >= Math.abs(minValue)){
            if(maxValue > driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].getMaxSpeed()){
                scaleValue = driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].getMaxSpeed()/maxValue;
            }
        }
        else if(Math.abs(maxValue) < Math.abs(minValue)){
            if(Math.abs(minValue) > driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].getMaxSpeed()){
                scaleValue = Math.abs(driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].getMaxSpeed()/ Math.abs(minValue));
            }
        }

        if(scaleValue != 1){
            for(int i = 0; i < finalVelocities.length; i ++){
                finalVelocities[i] *= scaleValue;
                Log.d("final velocity" + i,"" + finalVelocities[i]);
            }
        }

        applyMotorVelocities(finalVelocities);
    }

    public double[] calculateTurnVelocitiesRelativeToMax(double percentOfMax){
        double[] velocities = new double[4];
        double maxWheelVelocity = driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].getMaxSpeed();
        if(Double.isNaN(percentOfMax)) percentOfMax = 0;
        double velocity = maxWheelVelocity * percentOfMax;
        //double velocity = rps*WHEEL_BASE_RADIUS*2.0*Math.PI;
        if(Double.isNaN(velocity)){
            velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = 0;
            velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = 0;
            velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = 0;
            velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = 0;
        }
        else {
            velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = velocity;
            velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -velocity;
            velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = velocity;
            velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -velocity;
        }
        return velocities;
    }

    public double [] calculateTurnVelocities(double rps){
        double[] velocities = new double[4];
        if(Double.isNaN(rps)) rps = 0;
        double velocity = rps*WHEEL_BASE_RADIUS*2.0* Math.PI;
        if(Double.isNaN(velocity)){
            velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = 0;
            velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = 0;
            velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = 0;
            velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = 0;
        }
        else {
            velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = velocity;
            velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -velocity;
            velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = velocity;
            velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -velocity;
        }
        return velocities;
    }
    public void turn(double rps) {
        double[] velocities = calculateTurnVelocities(rps);
        applyMotorVelocities(velocities);
    }

    public double getDistanceFromHeading(double targetAngle){
        double distanceFromHeading = orientation.getOrientation() - targetAngle;
        if(distanceFromHeading > 180) distanceFromHeading -= 360;
        else if(distanceFromHeading < -180) distanceFromHeading += 360;
        return distanceFromHeading;
    }

    public void turnToHeading(double desiredHeading, LinearOpMode mode) {
        turnController.setSp(0);
        double curHeading = orientation.getOrientation() % 360;
        double rps;
        double distanceFromHeading = 0;
        distanceFromHeading = desiredHeading - curHeading;
        if(distanceFromHeading > 180) distanceFromHeading -= 360;
        else if(distanceFromHeading < -180) distanceFromHeading += 360;
        if(distanceFromHeading >= 0 && distanceFromHeading <= 180) {
            while(Math.abs(distanceFromHeading) > HEADING_THRESHOLD && mode.opModeIsActive()) {
                //heading always positive
                rps = turnController.calculatePID(distanceFromHeading);
                turn(-rps);
                mode.sleep(5);
                curHeading = orientation.getOrientation();
                distanceFromHeading = desiredHeading - curHeading;
                if(distanceFromHeading > 180) distanceFromHeading = distanceFromHeading - 360;
                else if(distanceFromHeading < -180) distanceFromHeading = 360 + distanceFromHeading;
            }
            brake();
        }
        else if((distanceFromHeading <= 360 && distanceFromHeading >= 180) || distanceFromHeading < 0) {
            while(Math.abs(distanceFromHeading) > HEADING_THRESHOLD && mode.opModeIsActive()) {
                //heading always positive
                rps = turnController.calculatePID(distanceFromHeading);
                turn(-rps);
                mode.sleep(5);
                curHeading = orientation.getOrientation();
                distanceFromHeading = desiredHeading - curHeading;
                if(distanceFromHeading > 180) distanceFromHeading -= 360;
                else if(distanceFromHeading < -180) distanceFromHeading += 360;
            }
            brake();
        }

    }

    public void turnToHeading(double desiredHeading, double tolerance, LinearOpMode mode){
        turnController.setSp(0);
        double curHeading = orientation.getOrientation() % 360;
        double rps;
        double distanceFromHeading = 0;
        distanceFromHeading = desiredHeading - curHeading;
        if(distanceFromHeading > 180) distanceFromHeading = distanceFromHeading - 360;
        else if(distanceFromHeading < -180) distanceFromHeading = 360 + distanceFromHeading;
        if(distanceFromHeading >= 0 && distanceFromHeading <= 180){
            while(Math.abs(distanceFromHeading) > tolerance && mode.opModeIsActive()){
                //heading always positive
                rps = turnController.calculatePID(distanceFromHeading);
                turn(-rps);
                mode.sleep(5);
                curHeading = orientation.getOrientation();
                distanceFromHeading = desiredHeading - curHeading;
                if(distanceFromHeading > 180) distanceFromHeading = distanceFromHeading - 360;
                else if(distanceFromHeading < -180) distanceFromHeading = 360 + distanceFromHeading;
            }
            brake();
        }

        else if((distanceFromHeading <= 360 && distanceFromHeading >= 180) || distanceFromHeading < 0){
            while(Math.abs(distanceFromHeading) > tolerance && mode.opModeIsActive()){
                //heading always positive
                rps = turnController.calculatePID(distanceFromHeading);
                turn(-rps);
                mode.sleep(5);
                curHeading = orientation.getOrientation();
                distanceFromHeading = desiredHeading - curHeading;
                if(distanceFromHeading > 180) distanceFromHeading = distanceFromHeading - 360;
                else if(distanceFromHeading < -180) distanceFromHeading = 360 + distanceFromHeading;
            }
            brake();
        }

    }

    public void setDrivePower(double power){
        double[] powers = new double[4];
        for(int i = 0; i < 4; i++){
            powers[i] = power;
        }
        applyMotorPowers(powers);
    }

    public void applyMotorVelocities(double [] velocities){
        for(MotorController m : driveMotors) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
    }

    public void applyMotorPowers(double [] powers){
        for(MotorController m : driveMotors) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(powers[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(powers[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(powers[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(powers[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
    }

    public void brake(){
        applyMotorVelocities(new double []{0,0,0,0});
    }

    public void stopNavigation(){
        shouldRun = false;
        for(int i =0; i < driveMotors.length; i ++){
            driveMotors[i].killMotorController();
        }
        orientation.stopIMU();
    }

    private void driveToLocation(Location startLocation, Location targetLocation, double desiredSpeed, LinearOpMode mode){
        double distanceToTravel = startLocation.distanceToLocation(targetLocation);
        double prevDistance = 0;
        double deltaX;
        double deltaY;
        double heading;
        double startHeading = restrictAngle(orientation.getOrientation(), targetLocation.getHeading(), mode);
        Log.d("Start heading", startHeading + "");
        double totalDistanceToTravel = distanceToTravel;
        while(mode.opModeIsActive() && distanceToTravel > LOCATION_DISTANCE_TOLERANCE /*&& prevDistance - distanceToTravel < LOCATION_DISTANCE_TOLERANCE*4*/) {
            prevDistance = distanceToTravel;
            distanceToTravel = startLocation.distanceToLocation(targetLocation); // start location is updated from the robot's current location (myLocation)
            Log.d("Distance to travel", "" + distanceToTravel);
            deltaX = targetLocation.getX() - startLocation.getX();
            deltaY = targetLocation.getY() - startLocation.getY();
            heading = Math.toDegrees(Math.atan2(deltaY, deltaX)) - 90;
            heading = 360 - heading;
            heading = (heading /*- orientation.getOrientation()*/) % 360;
            if (heading >= 360) heading -= 360;
            if (heading < 0) heading += 360;
            Log.d("Drive Heading", ""+heading);
            double curOrientation = restrictAngle(orientation.getOrientation(), 180, mode);
            Log.d("Robot Heading", ""+curOrientation);
            double fracOfDistance = distanceToTravel / (totalDistanceToTravel);
            if(fracOfDistance > 1) fracOfDistance = 1;
            turnController.setSp(/*(1-fracOfDistance)*(targetLocation.getHeading()) + (fracOfDistance)*(startHeading)*/targetLocation.getHeading());
            correctedDriveOnHeadingIMU(heading /*- curOrientation*/, desiredSpeed, 10, mode);
        }
        brake();
        Log.d("Location: ", "REACHED!");
//        driveToLocation(startLocation, targetLocation, desiredSpeed, 10000, mode);
    }

    private void driveToLocation(Location startLocation, Location targetLocation, double desiredSpeed, double secToQuit, LinearOpMode mode){
        double distanceToTravel = startLocation.distanceToLocation(targetLocation);
        double prevDistance = 0;
        double deltaX;
        double deltaY;
        double heading;
        double startHeading = restrictAngle(orientation.getOrientation(), targetLocation.getHeading(), mode);
        Log.d("Start heading", startHeading + "");
        double totalDistanceToTravel = distanceToTravel;
        long startTime = System.currentTimeMillis();
        while(mode.opModeIsActive() && distanceToTravel > LOCATION_DISTANCE_TOLERANCE && System.currentTimeMillis() - startTime < secToQuit*1000) {
            prevDistance = distanceToTravel;
            distanceToTravel = startLocation.distanceToLocation(targetLocation); // start location is updated from the robot's current location (myLocation)
            Log.d("Distance to travel", "" + distanceToTravel);
            deltaX = targetLocation.getX() - startLocation.getX();
            deltaY = targetLocation.getY() - startLocation.getY();
            heading = Math.toDegrees(Math.atan2(deltaY, deltaX)) - 90;
            heading = 360 - heading;
            heading = (heading - orientation.getOrientation()) % 360;
            if (heading >= 360) heading -= 360;
            if (heading < 0) heading += 360;
            Log.d("Heading", ""+heading);
            double curOrientation = restrictAngle(orientation.getOrientation(), 180, mode);
            double fracOfDistance = distanceToTravel / (totalDistanceToTravel);
            if(fracOfDistance > 1) fracOfDistance = 1;
            turnController.setSp(/*(1-fracOfDistance)*(targetLocation.getHeading()) + (fracOfDistance)*(startHeading)*/targetLocation.getHeading());
            correctedDriveOnHeadingIMU(heading - curOrientation, desiredSpeed, 10, mode);
        }
        brake();
    }

    public void driveToLocationPID(Location startLocation, Location targetLocation, double desiredSpeed, double locationTolerance, LinearOpMode mode) {
        if(targetLocation.getHeading() == Double.MIN_VALUE) targetLocation.setHeading(startLocation.getHeading()); // this should help to avoid having heading issues

        xPositionController.setSp(0);
        yPositionController.setSp(0);
        turnController.setSp(0);

        Location actualStartLocation = new Location(startLocation);
        double turnGain = 60; // was 45
        double decel = 50;
        double accel = 80;
        double xDist =  targetLocation.getX() - startLocation.getX();
        double yDist = targetLocation.getY() - startLocation.getY();
        double distToTravel = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
        double distTravelled = 0;
        double velocity = 0;
        double distToHeading = targetLocation.getHeading() - startLocation.getHeading();
        distToHeading = restrictAngle(distToHeading, 0, mode);
        long startTime = System.currentTimeMillis();
        while (mode.opModeIsActive() && (Math.abs(xDist) > locationTolerance || Math.abs(yDist) > locationTolerance || Math.abs(distToHeading) > HEADING_THRESHOLD)) {
            xDist = targetLocation.getX() - startLocation.getX();
            yDist = targetLocation.getY() - startLocation.getY();
            distToHeading = targetLocation.getHeading() - startLocation.getHeading();
            distToHeading = restrictAngle(distToHeading, 0, mode);
            distTravelled = startLocation.distanceToLocation(actualStartLocation);

//            This wasn't really logging and I got confused
            Log.d("Xdist", "" + xDist);
            Log.d("Ydist", "" + yDist);

            double timeToStop = velocity / decel;
            double distToStop = 0.5 * velocity * timeToStop;
            distToStop += distToTravel * STOPPING_DISTANCE_FACTOR;
            Log.d("Time to stop: ", ""+timeToStop);
            Log.d("Dist to stop: ", ""+distToStop);
            Log.d("Dist to travel: ", ""+distToTravel);
            Log.d("Dist travelled: ", ""+distTravelled);
            Log.d("Velocity: ", ""+velocity);
            mode.telemetry.addData("Velocity", velocity);
            mode.telemetry.update();
            if (distTravelled >= distToTravel - distToStop) {
                Log.d("Decelerating", "...");
                velocity = velocity - decel * (System.currentTimeMillis() - startTime) / 1000.0;
                if(velocity > desiredSpeed) velocity = desiredSpeed;
                if(velocity < 7.5) {
                    velocity = 7.5; // limit to 15 to allow PID to take over
                    Log.d("PID being used", "...");
                }
            } else {
                Log.d("Accelerating/Constant", "...");
                velocity = velocity + accel * (System.currentTimeMillis() - startTime) / 1000.0;
                if(velocity > desiredSpeed) velocity = desiredSpeed;
            }
            startTime = System.currentTimeMillis();

            double xCorrection = xPositionController.calculatePID(-xDist);
            Log.d("xCorrection: ", xCorrection + "");
            double yCorrection = yPositionController.calculatePID(-yDist);
            Log.d("yCorrection: ", yCorrection + "");
            double turnCorrection = turnController.calculatePID(-distToHeading) * turnGain;
            Log.d("turnCorrection: ", turnCorrection + "");

            double[] motorVelocities = new double[4];
            if(startLocation.getHeading() <= 45 && startLocation.getHeading() > -45) { // 0
                Log.d("dir", "0");
                motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = yCorrection + xCorrection;
                motorVelocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = yCorrection - xCorrection;
                motorVelocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = yCorrection + xCorrection;
                motorVelocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = yCorrection - xCorrection;

            } else if(startLocation.getHeading() <= 135 && startLocation.getHeading() > 45) { // 90
                Log.d("dir", "90");
                motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection + xCorrection;
                motorVelocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = yCorrection + xCorrection;
                motorVelocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection + xCorrection;
                motorVelocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = yCorrection + xCorrection;

            } else if(startLocation.getHeading() <= -135 || startLocation.getHeading() > 135) { // 180 or -180
                Log.d("dir", "180");
                motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection - xCorrection;
                motorVelocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection + xCorrection;
                motorVelocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection - xCorrection;
                motorVelocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection + xCorrection;

            } else if(startLocation.getHeading() <= -45 && startLocation.getHeading() > -135) { // -90
                Log.d("dir", "-90");
                motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = yCorrection - xCorrection;
                motorVelocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection - xCorrection;
                motorVelocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = yCorrection - xCorrection;
                motorVelocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection - xCorrection;

            }

            double maxValue = motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR], minValue = motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR];

            for (double d : motorVelocities) {
                if (maxValue < d) maxValue = d;
                else if (minValue > d) minValue = d;
            }

            double toScaleBy = (Math.abs(maxValue) > Math.abs(minValue)) ? Math.abs(velocity / maxValue) : Math.abs(velocity / minValue);
            for (int i = 0; i < motorVelocities.length; i++) motorVelocities[i] *= toScaleBy;

            motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] += turnCorrection;
            motorVelocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= turnCorrection;
            motorVelocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= turnCorrection;
            motorVelocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] += turnCorrection;

            applyMotorVelocities(motorVelocities);
        }
        brake();
    }

    public void driveToLocationPID(Location startLocation, Location targetLocation, double desiredSpeed, double locationTolerance, double secToQuit, LinearOpMode mode) {
        xPositionController.setSp(0);
        yPositionController.setSp(0);
        turnController.setSp(0);

        Location actualStartLocation = new Location(startLocation);
        double turnGain = 60;
        double decel = 32;
        double accel = 40;
        double xDist =  targetLocation.getX() - startLocation.getX();
        double yDist = targetLocation.getY() - startLocation.getY();
        double distToTravel = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
        double distTravelled = 0;
        double velocity = 0;
        double distToHeading = targetLocation.getHeading() - startLocation.getHeading();
        distToHeading = restrictAngle(distToHeading, 0, mode);
        long startTime = System.currentTimeMillis(), loopStartTime = System.currentTimeMillis();
        while (mode.opModeIsActive() && (Math.abs(xDist) > locationTolerance || Math.abs(yDist) > locationTolerance || Math.abs(distToHeading) > HEADING_THRESHOLD) && System.currentTimeMillis() - loopStartTime < secToQuit*1000) {
            xDist = targetLocation.getX() - startLocation.getX();
            yDist = targetLocation.getY() - startLocation.getY();
            distToHeading = targetLocation.getHeading() - startLocation.getHeading();
            distToHeading = restrictAngle(distToHeading, 0, mode);
            distTravelled = startLocation.distanceToLocation(actualStartLocation);

//            This wasn't really logging and I got confused
            Log.d("Xdist", "" + xDist);
            Log.d("Ydist", "" + yDist);

            double timeToStop = velocity / decel;
            double distToStop = 0.5 * velocity * timeToStop;
            distToStop += distToTravel * STOPPING_DISTANCE_FACTOR;
            Log.d("Time to stop: ", ""+timeToStop);
            Log.d("Dist to stop: ", ""+distToStop);
            Log.d("Dist to travel: ", ""+distToTravel);
            Log.d("Dist travelled: ", ""+distTravelled);
            Log.d("Velocity: ", ""+velocity);
            mode.telemetry.addData("Velocity", velocity);
            mode.telemetry.update();
            if (distTravelled >= distToTravel - distToStop) {
                Log.d("Decelerating", "...");
                velocity = velocity - decel * (System.currentTimeMillis() - startTime) / 1000.0;
                if(velocity > desiredSpeed) velocity = desiredSpeed;
                if(velocity < 7.5) {
                    velocity = 7.5; // limit to 15 to allow PID to take over
                    Log.d("PID being used", "...");
                }
            } else {
                Log.d("Accelerating/Constant", "...");
                velocity = velocity + accel * (System.currentTimeMillis() - startTime) / 1000.0;
                if(velocity > desiredSpeed) velocity = desiredSpeed;
            }
            startTime = System.currentTimeMillis();

            double xCorrection = xPositionController.calculatePID(-xDist);
            Log.d("xCorrection: ", xCorrection + "");
            double yCorrection = yPositionController.calculatePID(-yDist);
            Log.d("yCorrection: ", yCorrection + "");
            double turnCorrection = turnController.calculatePID(-distToHeading) * turnGain;
            Log.d("turnCorrection: ", turnCorrection + "");

            double[] motorVelocities = new double[4];
            if(startLocation.getHeading() <= 45 && startLocation.getHeading() > -45) { // 0
                Log.d("dir", "0");
                motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = yCorrection + xCorrection;
                motorVelocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = yCorrection - xCorrection;
                motorVelocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = yCorrection + xCorrection;
                motorVelocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = yCorrection - xCorrection;

            } else if(startLocation.getHeading() <= 135 && startLocation.getHeading() > 45) { // 90
                Log.d("dir", "90");
                motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection + xCorrection;
                motorVelocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = yCorrection + xCorrection;
                motorVelocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection + xCorrection;
                motorVelocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = yCorrection + xCorrection;

            } else if(startLocation.getHeading() <= -135 || startLocation.getHeading() > 135) { // 180 or -180
                Log.d("dir", "180");
                motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection - xCorrection;
                motorVelocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection + xCorrection;
                motorVelocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection - xCorrection;
                motorVelocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection + xCorrection;

            } else if(startLocation.getHeading() <= -45 && startLocation.getHeading() > -135) { // -90
                Log.d("dir", "-90");
                motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = yCorrection - xCorrection;
                motorVelocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection - xCorrection;
                motorVelocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = yCorrection - xCorrection;
                motorVelocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = -yCorrection - xCorrection;

            }

            double maxValue = motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR], minValue = motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR];

            for (double d : motorVelocities) {
                if (maxValue < d) maxValue = d;
                else if (minValue > d) minValue = d;
            }

            double toScaleBy = (Math.abs(maxValue) > Math.abs(minValue)) ? Math.abs(velocity / maxValue) : Math.abs(velocity / minValue);
            for (int i = 0; i < motorVelocities.length; i++) motorVelocities[i] *= toScaleBy;

            motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] += turnCorrection;
            motorVelocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= turnCorrection;
            motorVelocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= turnCorrection;
            motorVelocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] += turnCorrection;

            applyMotorVelocities(motorVelocities);
        }
        brake();
    }

    public void driveToLocationPID(Location startLocation, Location targetLocation, double desiredSpeed, LinearOpMode mode) {
        driveToLocationPID(startLocation, targetLocation, desiredSpeed, LOCATION_DISTANCE_TOLERANCE, mode);
    }

    public void driveToLocationPID(Location targetLocation, double desiredSpeed, LinearOpMode mode) {
        driveToLocationPID(myLocation, targetLocation, desiredSpeed, mode);
    }

    public void driveToLocationPID(Location targetLocation, double desiredSpeed, double locationTolerance, LinearOpMode mode) {
        driveToLocationPID(myLocation, targetLocation, desiredSpeed, locationTolerance, mode);
    }

    public void driveToLocationPID(Location targetLocation, double desiredSpeed, double locationTolerance, double secToQuit, LinearOpMode mode) {
        driveToLocationPID(myLocation, targetLocation, desiredSpeed, locationTolerance, secToQuit, mode);
    }

    public void driveToLocation(Location targetLocation, double desiredSpeed, LinearOpMode mode){
        driveToLocation(myLocation, targetLocation, desiredSpeed, mode);
    }

    public void driveToLocation(Location targetLocation, double desiredSpeed, double secToQuit, LinearOpMode mode){
        driveToLocation(myLocation, targetLocation, desiredSpeed, secToQuit, mode);
    }

    public double getDistanceFrom(Location location) {
        // TODO
//        This is called the distance formula, Jordan. Remember the song?
        return Math.sqrt(Math.pow(location.getX() - myLocation.getX(), 2) + Math.pow(location.getY() - myLocation.getY(), 2));
    }

    public void navigatePath(Location[] path, double desiredSpeed, LinearOpMode mode) {
        for(int i = 0; i < path.length; i++) {
            driveToLocation(path[i], desiredSpeed, mode);
            Log.d("Target Location", path[i]+"");
            Log.d("Actual Location", getRobotLocation()+"");
            mode.sleep(1000);
        }
    }

    public void navigatePath(Location[] path, double desiredSpeed, double[] secToQuit, LinearOpMode mode) {
        for(int i = 0; i < path.length; i++) {
            driveToLocation(path[i], desiredSpeed, secToQuit[i], mode);
        }
    }

    public void navigatePathPID(Location[] path, double desiredSpeed, LinearOpMode mode) {
        for(Location toTravelTo : path) {
            driveToLocationPID(toTravelTo, desiredSpeed, mode);
        }
    }

    public HeadingVector[] getWheelVectors(){
        double [] deltaWheelPositions = {0,0,0,0};
        for(int i = 0; i < driveMotors.length; i ++){
            double a = driveMotors[i].getInchesFromStart();
            Log.d("Last Motor Pos Inches:", lastMotorPositionsInInches[i]+"");
            deltaWheelPositions[i] = a - lastMotorPositionsInInches[i];
            lastMotorPositionsInInches[i] = a;
        }
        //updateLastMotorPositionsInInches();
        HeadingVector[] vectors = new HeadingVector[4];
        for(int i = 0; i < vectors.length; i++){
            vectors[i] = new HeadingVector();
        }
        vectors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].calculateVector(FL_WHEEL_HEADING_OFFSET,deltaWheelPositions[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        vectors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].calculateVector(FR_WHEEL_HEADING_OFFSET,deltaWheelPositions[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
        vectors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].calculateVector(BL_WHEEL_HEADING_OFFSET,deltaWheelPositions[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        vectors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].calculateVector(BR_WHEEL_HEADING_OFFSET,deltaWheelPositions[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
        return vectors;
    }

    private double restrictAngle(double angleToChange, double referenceAngle, LinearOpMode mode) {
        while(mode.opModeIsActive() && angleToChange < referenceAngle - 180) angleToChange += 360;
        while (mode.opModeIsActive() && angleToChange > referenceAngle + 180) angleToChange -= 360;
        return angleToChange;
    }

    private void populateHashmaps() {
        updateLocationInformation[Q1] = new HashMap<>();
        updateLocationInformation[Q2] = new HashMap<>();
        updateLocationInformation[Q3] = new HashMap<>();
        updateLocationInformation[Q4] = new HashMap<>();

        updateLocationInformation[Q1].put(NORTH, new int[] {RIGHT_SENSOR, DRIVE_BASE});
        updateLocationInformation[Q1].put(EAST, new int[] {DRIVE_BASE, LEFT_SENSOR});
        updateLocationInformation[Q1].put(SOUTH, new int[] {LEFT_SENSOR, BACK_SENSOR});
        updateLocationInformation[Q1].put(WEST, new int[] {BACK_SENSOR, RIGHT_SENSOR});

        updateLocationInformation[Q2].put(NORTH, new int[] {LEFT_SENSOR, DRIVE_BASE});
        updateLocationInformation[Q2].put(EAST, new int[] {BACK_SENSOR, LEFT_SENSOR});
        updateLocationInformation[Q2].put(SOUTH, new int[] {RIGHT_SENSOR, BACK_SENSOR});
        updateLocationInformation[Q2].put(WEST, new int[] {DRIVE_BASE, RIGHT_SENSOR});

        updateLocationInformation[Q3].put(NORTH, new int[] {LEFT_SENSOR, BACK_SENSOR});
        updateLocationInformation[Q3].put(EAST, new int[] {BACK_SENSOR, RIGHT_SENSOR});
        updateLocationInformation[Q3].put(SOUTH, new int[] {RIGHT_SENSOR, DRIVE_BASE});
        updateLocationInformation[Q3].put(WEST, new int[] {DRIVE_BASE, LEFT_SENSOR});

        updateLocationInformation[Q4].put(NORTH, new int[] {RIGHT_SENSOR, BACK_SENSOR});
        updateLocationInformation[Q4].put(EAST, new int[] {DRIVE_BASE, RIGHT_SENSOR});
        updateLocationInformation[Q4].put(SOUTH, new int[] {LEFT_SENSOR, DRIVE_BASE});
        updateLocationInformation[Q4].put(WEST, new int[] {BACK_SENSOR, LEFT_SENSOR});
    }
}
