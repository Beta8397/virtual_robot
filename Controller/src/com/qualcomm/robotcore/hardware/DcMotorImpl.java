package com.qualcomm.robotcore.hardware;

import java.util.Random;

/**
 * Implementation of the DcMotor interface.
 */
public class DcMotorImpl implements DcMotor {
    public final MotorType MOTOR_TYPE;
    private final Random random = new Random();
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private Direction direction = Direction.FORWARD;

    //power is the requested speed, normalized to the -1 to +1 range
    private double power = 0.0;

    //speed is the actual speed, normalized to the -1 to +1 range
    private double speed = 0.0;

    //actual position of motor
    private double actualPosition = 0.0;

    //position to use as baseline for encoder tick calculation
    private double encoderBasePosition = 0.0;

    private double randomErrorFrac = 0.0;
    private double systematicErrorFrac = 0.0;
    private double inertia;

    /**
     * For internal use only.
     * @param motorType
     */
    public DcMotorImpl(MotorType motorType){
        MOTOR_TYPE = motorType;
    }

    /**
     * Set mode of operation
     * @param mode
     */
    public synchronized void setMode(RunMode mode){
        this.mode = mode;
        if (mode == RunMode.STOP_AND_RESET_ENCODER){
            power = 0.0;
            encoderBasePosition = actualPosition;
        }
    }

    /**
     * Get mode of operation
     * @return mode
     */
    public synchronized RunMode getMode(){ return mode; }

    /**
     * Set logical direction
     * @param direction the direction to set for this motor
     */
    public synchronized void setDirection(Direction direction){ this.direction = direction; }

    /**
     * Get logical direction
     * @return direction
     */
    public synchronized Direction getDirection(){ return direction; }

    /**
     * Get requested power
     * @return
     */
    public synchronized double getPower(){ return power; }

    /**
     * Set requested power
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     */
    public synchronized void setPower(double power){
        this.power = Math.max(-1, Math.min(1, power));
    }

    /**
     * Get current position (as number of encoder ticks)
     * @return number of encoder ticks
     */
    public synchronized int getCurrentPosition(){
        int result = (int)Math.floor(actualPosition - encoderBasePosition);
        return direction == Direction.FORWARD && MOTOR_TYPE.REVERSED ||
                direction == Direction.REVERSE && !MOTOR_TYPE.REVERSED ? -result : result;
    }

    /**
     * For internal use only.
     * @return
     */
    public synchronized double getActualPosition(){ return actualPosition; }

    /**
     * For internal use only.
     * Updates motor speed based on current speed, power, and inertia. Then, uses motor speed to update position.
     * @param milliseconds number of milliseconds since last update
     * @return change in actualPosition
     */
    public synchronized double update(double milliseconds){
        if (mode == RunMode.RUN_TO_POSITION || mode == RunMode.STOP_AND_RESET_ENCODER) return 0.0;
        speed = speed + (1.0 - inertia) * (power - speed);
        double positionChange = speed * MOTOR_TYPE.MAX_TICKS_PER_SECOND * milliseconds / 1000.0;
        positionChange *= (1.0 + systematicErrorFrac + randomErrorFrac * random.nextGaussian());
        if (direction == Direction.FORWARD && MOTOR_TYPE.REVERSED ||
                direction == Direction.REVERSE && !MOTOR_TYPE.REVERSED) positionChange = -positionChange;
        actualPosition += positionChange;
        return positionChange;
    }

    /**
     * For internal use only.
     * @param rdmErrFrac
     */
    public synchronized void setRandomErrorFrac(double rdmErrFrac){
        randomErrorFrac = rdmErrFrac;
    }

    /**
     * For internal use only.
     * @param sysErrFrac
     */
    public synchronized void setSystematicErrorFrac(double sysErrFrac) { systematicErrorFrac = sysErrFrac; }

    /**
     * For internal use only.
     * @param in
     */
    public synchronized void setInertia(double in){
        if (in < 0) inertia = 0.0;
        else if (in > 0.99) inertia = 0.99;
        else inertia = in;
    }

    //For internal use only: for stopping and resetting motor between op mode runs
    public synchronized void stopAndReset(){
        power = 0.0;
        speed = 0.0;
        actualPosition = 0.0;
        encoderBasePosition = 0.0;
        direction = Direction.FORWARD;
    }

}
