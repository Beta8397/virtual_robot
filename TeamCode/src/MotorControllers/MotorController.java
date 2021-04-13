//use the standard DC Motor Controller but create wrapper for it to more easily control velocities, distances, etc.

package MotorControllers;

import Misc.ConfigFile;
import Misc.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.IOException;
import java.io.InputStream;

/**
 * Created by root on 11/11/17.
 */

/*
    A class to more easily and reliably control our DC motors
 */
public class MotorController extends Thread {
    //config file values
    private long maxTicksPerSecond = 0;
    private long ticksPerRevolution = 0;
    private double wheelDiameterInInches = 0;
    private double ticksPerDegree = 0;
    //user set and program updated variables
    private long currentTicksPerSec = 0;
    private long curTickLocation = 0;
    DcMotor motor;
    private String logTag = "";
    private boolean shouldLog = false;
    private boolean takenStartValue = false;
    private volatile boolean shouldRun = false;
    MotorTachometer tachometer;
    PIDController holdController;
    private double initialDegree = 0;
    private long LOOP_MILLIS = 200;
    private int startPos = 0;

    HardwareMap hardwareMap;

    public MotorController(DcMotor m, String configFileLoc, HardwareMap hw) throws IOException {
        hardwareMap = hw;
        motor = m;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //read the config file. If error received, fail entire initialization
        if (readConfig(configFileLoc) != 1){
            logError("MotorController Error","Config File Read Failed! Killing self!");
            shouldRun = false;
            throw new IOException("Failed to parse Motor Config File: " + configFileLoc);
        }
        tachometer = new MotorTachometer(m, ticksPerRevolution, MotorTachometer.RPS_SMOOTHER.NONE);
        shouldRun = true;
        logDebug("Ticks per rev", Double.toString(ticksPerRevolution));
        new Thread(new Runnable() {
            @Override
            public void run() {
                long loopStartMillis;
                while(shouldRun) {
                    //get start time
                    loopStartMillis = System.currentTimeMillis();
                    //update for runtime
                    updateData();
                    long remainingTime = LOOP_MILLIS - (System.currentTimeMillis() - loopStartMillis);
                    if (remainingTime > 0) safetySleep(remainingTime);
                }
            }
        });
    }

    public MotorController(String motorName, String configFileLoc, HardwareMap hw) throws IOException {
        this(hw.dcMotor.get(motorName), configFileLoc, hw);
    }

    public MotorController(String motorName, String configFileLoc, String debugTag, HardwareMap hw) throws IOException {
        this(hw.dcMotor.get(motorName), configFileLoc, hw);
        shouldLog = true;
        logTag = debugTag;
    }

    public MotorController(String motorName, HardwareMap hardwareMap) { motor = hardwareMap.dcMotor.get(motorName); }

    public void setDefaultTicksPerDegree() {
        ticksPerDegree = ticksPerRevolution / 360.0;
    }

    public void setTicksPerDegree(double ticksPerDegree) {
        this.ticksPerDegree = ticksPerDegree;
    }

    public DcMotor.RunMode getMotorRunMode() { return motor.getMode(); }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior b) { motor.setZeroPowerBehavior(b); }

    public void setDirection(DcMotor.Direction dir) {
        motor.setDirection(dir);
    }

    public void setMode(DcMotor.RunMode mode) {
        try {
            if (getMotorRunMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                if (motor.isBusy()) {
                    setMotorPower(0);
                    Log.d("Motor issue" ,"Still busy....");
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
            motor.setMode(mode);
        } catch (Exception e) {
            Log.d("Error!", e.toString());
        }
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public int getTargetPosition() { return motor.getTargetPosition(); }

    private void safetySleep(long time) {
        long start = System.currentTimeMillis();
        while(System.currentTimeMillis() - start < time && shouldRun);
    }

    public void killMotorController() {
        brake();
        shouldRun = false;
    }

    private void updateData() {
        //get new RPS
        double currentRPS = 0;
        try {
            currentRPS = tachometer.getRotationsPerSecond();
        } catch(Exception e){
            logError("MotorController Err",e.toString());
            shouldRun = false;
            throw e;
        }
        currentTicksPerSec = (long)(currentRPS * ticksPerRevolution + .5);
        //update position
        curTickLocation = motor.getCurrentPosition();
    }

    private int readConfig(String fileLoc) {
        InputStream stream = null;
        try {
            stream = ConfigFile.open(hardwareMap, fileLoc);
        } catch(Exception e) {
            logError("Error: ", e.toString());
            throw new RuntimeException(e);
        }
        JsonConfigReader reader = new JsonConfigReader(stream);
        try {
            //ticksPerRevolution = reader.getLong("TICKS_PER_REV");
            ticksPerRevolution = (long) motor.getMotorType().getTicksPerRev();
            wheelDiameterInInches = reader.getDouble("WHEEL_DIAMETER");
            ticksPerDegree = reader.getDouble("TICKS_PER_DEGREE");
            initialDegree = reader.getDouble("INITIAL_DEGREE");
            //double maxRPS = reader.getDouble("MAX_RPS");
            double maxRPS = motor.getMotorType().getMaxRPM();
            //maxTicksPerSecond = (long)(maxRPS * ticksPerRevolution + .5);
            maxTicksPerSecond = (long) motor.getMotorType().getAchieveableMaxTicksPerSecondRounded();
            holdController = new PIDController(reader.getDouble("HOLD_KP"), reader.getDouble("HOLD_KI"), reader.getDouble("HOLD_KD"));
            holdController.setIMax(reader.getDouble("HOLD_I_MAX"));
        } catch(Exception e) {
            logError(logTag + " MotorController Error", "Config File Read Fail: " + e.toString());
            return 0;
        }
        return 1;
    }

    public double getMotorPower(){
        return motor.getPower();
    }

    public long getCurrentTicksPerSecond(){
        return currentTicksPerSec;
    }

    public double getCurrentInchesPerSecond() {
        return (double) getCurrentTicksPerSecond() / (double) (ticksPerRevolution) * wheelDiameterInInches * Math.PI;
    }

    public double getCurrentRPS() {
        return (double)getCurrentTicksPerSecond() / (double)ticksPerRevolution;
    }

    public long getCurrentTick() { return motor.getCurrentPosition(); }

    public double getInchesFromStart(){
        return (double) getCurrentTick() / (double)(ticksPerRevolution) * wheelDiameterInInches * Math.PI;
    }

    public double getDegree() {
        return getCurrentTick() / ticksPerDegree + initialDegree;
    }

    public void setTicksPerSecondVelocity(long ticksPerSec) {
        if(getMotorRunMode() != DcMotor.RunMode.RUN_USING_ENCODER) setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        takenStartValue = false;
        //check for flip in sign, if so, reset our pid controller
        //power is a function of ticksPerSec/maxAcheivable ticks per second
        //Log.d("MotorVel", "" + ticksPerSec + " T/s");
        try {
            motor.setPower((double) ticksPerSec / motor.getMotorType().getAchieveableMaxTicksPerSecondRounded());
        } catch (Exception e){
            logError("Motorcontroller Error", "SetTicksPerSecondVelocity: " + e.toString());
            shouldRun = false;
            throw e;
        }
        //Log.d("MotorPow", "" + getMotorPower() + " %");

    }

    public void setInchesPerSecondVelocity(double inchesPerSec) {
        long ticksPerSec = (long)(inchesPerSec/(wheelDiameterInInches* Math.PI)*ticksPerRevolution + .5);
        setTicksPerSecondVelocity(ticksPerSec);
    }

    public double convertTicksToInches(long ticks){
        return (double) ticks / ticksPerRevolution * Math.PI  * wheelDiameterInInches;
    }

    public void holdPosition() {
//        if(getMotorRunMode() != DcMotor.RunMode.RUN_TO_POSITION){
//            motor.setPower(0);
//            try {
//                sleep(20);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//            motor.setTargetPosition(motor.getCurrentPosition());
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motor.setPower(0.6);
//        }

        Log.d("Hold Position", "Start");

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!takenStartValue) {
            startPos = motor.getCurrentPosition();
            takenStartValue = true;
            Log.d("Hold Position", "Start value taken");
        }
        holdController.setSp(0);
        int distToPos = motor.getCurrentPosition() - startPos;
        Log.d("Hold Position", "KP" + holdController.getP());
        Log.d("Hold Position", "Distance to position" + distToPos);
        double motorPower = holdController.calculatePID(distToPos);
        Log.d("Hold Position", "Motor Power" + motorPower);
        motor.setPower(motorPower);
//        motor.setPower(0);
    }

    public double getWheelDiameterInInches(){
        return wheelDiameterInInches;
    }

    public int getTicksPerRevolution() { return (int)ticksPerRevolution; }

    public void setMotorPower(double power) {
        takenStartValue = false;
        motor.setPower(power);
    }

    public void brake() {
        takenStartValue = false;
        if(getMotorRunMode() == DcMotor.RunMode.RUN_TO_POSITION)
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }

    public void setPositionInches(double positionInInches) {
        //go ahead and set mode
        motor.setPower(0);
        int positionInTicks = (int)(positionInInches/(wheelDiameterInInches* Math.PI)*ticksPerRevolution);
        logDebug("Desired tick", Double.toString(positionInTicks));
        motor.setTargetPosition(positionInTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPositionTicks(int tick) { motor.setTargetPosition(tick); }

    public void setPositionDegrees(double deg) {
        brake();
        int targetTick = (int)(deg*ticksPerDegree);
        motor.setTargetPosition(targetTick);
        if(getMotorRunMode() != DcMotor.RunMode.RUN_TO_POSITION) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPositionDegrees(double deg, double power) {
        brake();
        int targetTick = (int)(deg*ticksPerDegree);
        motor.setTargetPosition(targetTick);
        if(getMotorRunMode() != DcMotor.RunMode.RUN_TO_POSITION) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    private void logDebug(String main, String sub){
        if(shouldLog){
            Log.d(logTag, main + ":" + sub);
        }
    }

    public double getMaxSpeed(){
        return convertTicksToInches(maxTicksPerSecond);
    }

    private void logError(String main, String sub){
        Log.d(motor.getClass().getSimpleName(), logTag + ":" + main + ":" + sub);
    }

    public DcMotor getMotor() { return motor; }
    public double getTicksPerDegree() { return ticksPerDegree; }
}