package MotorControllers;

import Misc.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by robotics on 8/4/17.
 */

/*
    A class that helps us keep up with the speed of the robot/wheels
 */
public class MotorTachometer {
    private double averageRPS = 0;
    private final double A = 0.3;
    private double rotationsPerSecond = 0;
    private DcMotor motor;
    private long ticksPerRev = 0;
    private long lastCallMillis = 0;
    private long lastTickLocation = 0;
    public static final long NEVEREST_20 = 560;
    private FiFoBuffer timeBuffer;
    private FiFoBuffer tickBuffer;
    private int bufferLength = 5;
    public enum RPS_SMOOTHER {
        NONE, COMPLIMENTARY_FILTER, AVERAGE_VELOCITY, FIFO_BUFFER
    }
    private RPS_SMOOTHER smootherMode = RPS_SMOOTHER.NONE;

    public MotorTachometer(DcMotor m, long t){
        ticksPerRev = t;
        motor = m;
        lastCallMillis = System.currentTimeMillis();
        lastTickLocation = m.getCurrentPosition();
    }

    public MotorTachometer(DcMotor m, long t, RPS_SMOOTHER smoother) {
        smootherMode = smoother;
        ticksPerRev = t;
        motor = m;
        try {
            lastCallMillis = System.currentTimeMillis();
            lastTickLocation = m.getCurrentPosition();
            timeBuffer = new FiFoBuffer(bufferLength);
            tickBuffer = new FiFoBuffer(bufferLength);
            if (smootherMode == RPS_SMOOTHER.FIFO_BUFFER) {
                for (int i = 0; i < bufferLength; i++) {
                    timeBuffer.add((long) 0);
                    Log.d("Break Buffer load: ", "");
                    tickBuffer.add((long) 0);
                }
            }
        }
        catch(Exception e){
            Log.e("Tachometer",e.toString());
        }
    }

    public double getRotationsPerSecond(){
        switch (smootherMode){
            case NONE:
                rotationsPerSecond = calculateRotationsPerSecondNoMode();
                break;
            case COMPLIMENTARY_FILTER:
                rotationsPerSecond = calculateRotationsPerSecondComplimentaryFilter();
                break;
            case AVERAGE_VELOCITY:
                rotationsPerSecond = calculateRotationsPerSecondAverageVelocity();
                break;
            case FIFO_BUFFER:
                rotationsPerSecond = calculateRotationsPerSecondFIFOBuffer();
                break;
        }
        updateBuffer();
        return Math.abs(rotationsPerSecond);
    }

    private void updateBuffer(){

    }

    private double calculateRotationsPerSecondNoMode() {
        long tempTime = System.currentTimeMillis();
        long tempTick = motor.getCurrentPosition();
        //Log.d("Ticks", Long.toString(tempTick));
        double deltaTime = (tempTime - lastCallMillis) / 1000.0; //convert to seconds
        //Log.d("D_T: ", Double.toString(deltaTime));
        long deltaTicks = tempTick - lastTickLocation;
        //Log.d("Delta ticks: ", Long.toString(deltaTicks));
        double rotationsPerSecond = Math.abs((double)deltaTicks/(double)ticksPerRev/deltaTime);
        if(deltaTicks == 0) rotationsPerSecond = 0;
        //Log.d("RPS: ", Double.toString(rotationsPerSecond));
        lastCallMillis = tempTime;
        lastTickLocation = tempTick;
        return rotationsPerSecond;
    }

    private double calculateRotationsPerSecondComplimentaryFilter(){
        double instantaneousRPS = calculateRotationsPerSecondNoMode();
        averageRPS = A*instantaneousRPS + (1-A)*averageRPS;
        Log.d("Tachometer instRPS", Double.toString(instantaneousRPS));
        Log.d("Tachometer avgRPS", Double.toString(averageRPS));

        return averageRPS;
    }

    private double calculateRotationsPerSecondAverageVelocity(){
        double instantaneousRPS = calculateRotationsPerSecondNoMode();
        averageRPS = (averageRPS+instantaneousRPS)/2.0;
        Log.d("Tachometer instRPS", Double.toString(instantaneousRPS));
        Log.d("Tachometer avgRPS", Double.toString(averageRPS));
        return averageRPS;
    }

    private double calculateRotationsPerSecondFIFOBuffer(){
        double instantaneousRPS = calculateRotationsPerSecondNoMode();
        long checkTime = timeBuffer.getLast();
        long checkTick = tickBuffer.getLast();
        //Log.d("Check Tick",Long.toString(checkTick));
        //Log.d("Now Tick",Long.toString(lastTickLocation));
        //Log.d("Now Time",Long.toString(lastCallMillis));
        //Log.d("Check Time",Long.toString(checkTime));
        double deltaTime = ((double)lastCallMillis - checkTime)/1000.0;
        double deltaTick = (double)lastTickLocation - checkTick;
        double rps = (double)deltaTick/(double)deltaTime/(double)ticksPerRev;
        timeBuffer.add(lastCallMillis);
        tickBuffer.add(lastTickLocation);
        return rps;
    }

}
