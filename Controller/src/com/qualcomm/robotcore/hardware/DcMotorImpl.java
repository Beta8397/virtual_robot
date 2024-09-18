package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.configuration.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Random;

/**
 * Implementation of the DcMotor interface.
 */
public class DcMotorImpl implements DcMotor {
    public final MotorType MOTOR_TYPE;
    public final MotorConfigurationType MOTOR_CONFIGURATION_TYPE;
    public final DcMotorControllerImpl controller;
    public final int portNumber;

    //Proportionate coefficient for RUN_TO_POSITION mode
    protected final double COEFF_PROPORTIONATE = 5.0;

    // Rotation offset (in rotations) for isBusy
    protected final double MAX_ROT_OFFSET = 0.005;

    //Target position for RUN_TO_POSITION mode
    private int targetPosition = 0;
    private boolean targetPositionIsSet = false;

    private final Random random = new Random();
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private Direction direction = Direction.FORWARD;

    //power is the requested speed, normalized to the -1 to +1 range
    private double power = 0.0;

    /*
     * actualSpeed is current motor "actual speed", normalized to the -1 to +1 range. This refers
     * to the rate at which the output shaft is rotating in the CLOCKWISE direction (when viewed from
     * the shaft end). This will be equal in magnitude to speed, but may differ in sign, depending
     * upon the values of direction and MOTOR_TYPE.reversed.
     */
    private double actualSpeed = 0.0;

    //actual position of motor
    private double actualPosition = 0.0;

    //position to use as baseline for encoder tick calculation
    private double encoderBasePosition = 0.0;

    private boolean supportsError = true;
    private boolean supportsInertia = true;
    private double randomErrorFrac = 0.0;
    private double systematicErrorFrac = 0.0;
    private double inertia;

    private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;

    // Physical limits on output shaft travel
    private double upperActualPositionLimit = 0;
    private double lowerActualPositionLimit = 0;
    private boolean upperPositionLimitEnabled = false;
    private boolean lowerPositionLimitEnabled = false;


    /**
     * For internal use only.
     * @param motorType
     */
    public DcMotorImpl(MotorType motorType, DcMotorControllerImpl controller, int portNumber){
        MOTOR_TYPE = motorType;
        MOTOR_CONFIGURATION_TYPE = new MotorConfigurationType(motorType);
        this.controller = controller;
        this.portNumber = portNumber;
        controller.setMotor(portNumber, this);
    }

    /**
     * For internal use only
     * @param motorType
     * @param supportsError  True, if motor is to be affected by random and systematic error.
     * @param supportsInertia   True, if motor is to be affected by inertia.
     */
    public DcMotorImpl(MotorType motorType, DcMotorControllerImpl controller, int portNumber, boolean supportsError, boolean supportsInertia){
        MOTOR_TYPE = motorType;
        MOTOR_CONFIGURATION_TYPE = new MotorConfigurationType(motorType);
        this.controller = controller;
        this.portNumber = portNumber;
        this.supportsInertia = supportsInertia;
        this.supportsError = supportsError;
        controller.setMotor(portNumber, this);
    }

    /**
     * Set mode of operation
     * @param mode
     */
    public synchronized void setMode(RunMode mode){
        this.mode = mode;
        power = 0.0;
        if (mode == RunMode.STOP_AND_RESET_ENCODER){
            encoderBasePosition = actualPosition;
        } else if (mode == RunMode.RUN_TO_POSITION){
            if (!targetPositionIsSet) {
                throw new ActionNotSupportedException("Target position must be set before entering RUN_TO_POSITION mode.");
            }
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
        if (mode != RunMode.STOP_AND_RESET_ENCODER) {
            this.power = Math.max(-1, Math.min(1, power));
        }
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
     * Updates motor speed based on current speed, power, and inertia. Then, uses average motor speed to update position.
     * @param milliseconds number of milliseconds since last update
     * @return change in actualPosition
     */
//    public synchronized double oldUpdate(double milliseconds){
//        double speedChange, avgSpeed;
//        if (mode == RunMode.STOP_AND_RESET_ENCODER) return 0.0;
//        else if (mode == RunMode.RUN_TO_POSITION){
//            double targetSpeed = COEFF_PROPORTIONATE * (double)(targetPosition - getCurrentPosition())
//                    / MOTOR_TYPE.MAX_TICKS_PER_SECOND;
//            double absPower = Math.abs(power);
//            targetSpeed = Math.max(-absPower, Math.min(targetSpeed, absPower));
//            speedChange = (1.0 - inertia) * (targetSpeed - speed);
//            avgSpeed = speed + speedChange / 2.0;
//            speed = speed + speedChange;
//        } else {
//            speedChange = (1.0 - inertia) * (power - speed);
//            avgSpeed = speed + speedChange / 2.0;
//            speed = speed + speedChange;
//        }
//        double positionChange = avgSpeed * MOTOR_TYPE.MAX_TICKS_PER_SECOND * milliseconds / 1000.0;
//        positionChange *= (1.0 + systematicErrorFrac + randomErrorFrac * random.nextGaussian());
//        if (direction == Direction.FORWARD && MOTOR_TYPE.REVERSED ||
//                direction == Direction.REVERSE && !MOTOR_TYPE.REVERSED) {
//            positionChange = -positionChange;
//            actualSpeed = -speed;
//        } else {
//            actualSpeed = speed;
//        }
//        actualPosition += positionChange;
//        return positionChange;
//    }

    /*
     * For internal use only.
     * New version of the update method that is compatible with position limits.
     * Updates motor speed based on current speed, power, and inertia. Then, uses average motor speed to update position.
     * @param milliseconds number of milliseconds since last update
     * @return change in actualPosition
     */
    public synchronized double update(double milliseconds){
        //if (mode == RunMode.STOP_AND_RESET_ENCODER) return 0.0;
        double effectiveInertia = inertia;
        if (zeroPowerBehavior == ZeroPowerBehavior.BRAKE && power == 0){
            effectiveInertia *= 0.01;
        }

        double tentativeActualSpeedChange, avgActualSpeed, tentativeActualSpeed, actualSpeedChange;
        boolean rev = direction == Direction.FORWARD && MOTOR_TYPE.REVERSED
                || direction == Direction.REVERSE && !MOTOR_TYPE.REVERSED;
        if (mode == RunMode.RUN_TO_POSITION){
            double actualTargetPosition = rev? -targetPosition : targetPosition;
            double actualTargetSpeed = COEFF_PROPORTIONATE
                    * (double)(actualTargetPosition - (actualPosition - encoderBasePosition))
                    / MOTOR_TYPE.MAX_TICKS_PER_SECOND;
            double absPower = Math.abs(power);
            actualTargetSpeed = Math.max(-absPower, Math.min(actualTargetSpeed, absPower));
            tentativeActualSpeedChange = (1.0 - effectiveInertia) * (actualTargetSpeed - actualSpeed);
            avgActualSpeed = (actualSpeed + tentativeActualSpeedChange / 2.0)
                    * (1.0 + systematicErrorFrac + randomErrorFrac * random.nextGaussian());
            actualSpeedChange = 2.0 * (avgActualSpeed - actualSpeed);
            tentativeActualSpeed = actualSpeed + actualSpeedChange;
        } else {
            double actualPower = rev? -power : power;
            tentativeActualSpeedChange = (1.0 - effectiveInertia) * (actualPower - actualSpeed);
            avgActualSpeed = (actualSpeed + tentativeActualSpeedChange / 2.0)
                    * (1.0 + systematicErrorFrac + randomErrorFrac * random.nextGaussian());
            actualSpeedChange = 2.0 * (avgActualSpeed - actualSpeed);
            tentativeActualSpeed = actualSpeed + actualSpeedChange;
        }

        double tentativeActualPositionChange = avgActualSpeed * MOTOR_TYPE.MAX_TICKS_PER_SECOND * milliseconds / 1000.0;
        double tentativeActualPosition = actualPosition + tentativeActualPositionChange;

        if (upperPositionLimitEnabled && tentativeActualPosition > upperActualPositionLimit){
            tentativeActualPosition = upperActualPositionLimit;
            tentativeActualSpeed = Math.min(0, tentativeActualSpeed);
        } else if (lowerPositionLimitEnabled && tentativeActualPosition < lowerActualPositionLimit){
            tentativeActualPosition = lowerActualPositionLimit;
            tentativeActualSpeed = Math.max(0, tentativeActualSpeed);
        }

        actualSpeed = tentativeActualSpeed;
        double positionChange = tentativeActualPosition - actualPosition;
        actualPosition = tentativeActualPosition;

        return positionChange;
    }

    /**
     * For internal use only -- The FTC SDK does not include a "getSpeed()" method in the DcMotorImpl class.
     * Related: see getVelocity methods of DcMotorEx
     * @return current motor speed, normalized -1 to 1
     */
    protected synchronized double getSpeed(){
        boolean rev = direction == Direction.FORWARD && MOTOR_TYPE.REVERSED
                || direction == Direction.REVERSE && !MOTOR_TYPE.REVERSED;
        return rev? -actualSpeed : actualSpeed;
    }

    /**
     * For internal use only -- the FTC SDK does not include a "getActualSpeed()" method in the DcMotorImpl class.
     * @return current motor actual speed, normalized -1 to 1.
     */
    protected synchronized double getActualSpeed() { return actualSpeed; }

    /**
     * For internal use only.
     * @param rdmErrFrac
     */
    public synchronized void setRandomErrorFrac(double rdmErrFrac){
        if (!supportsError) return;
        randomErrorFrac = rdmErrFrac;
    }

    /**
     * For internal use only.
     * @param sysErrFrac
     */
    public synchronized void setSystematicErrorFrac(double sysErrFrac) {
        if (!supportsError) return;
        systematicErrorFrac = sysErrFrac; }

    /**
     * For internal use only.
     * @param in
     */
    public synchronized void setInertia(double in){
        if (!supportsInertia) return;
        if (in < 0) inertia = 0.0;
        else if (in > 0.99) inertia = 0.99;
        else inertia = in;
    }

    //For internal use only: for stopping and resetting motor between op mode runs
    public synchronized void stopAndReset(){
        power = 0.0;
        actualSpeed = 0.0;
        direction = Direction.FORWARD;
        mode = RunMode.RUN_WITHOUT_ENCODER;
        targetPositionIsSet = false;
    }

    //Set target position for RUN_TO_POSITION mode
    public synchronized void setTargetPosition(int pos){
        targetPosition = pos;
        targetPositionIsSet = true;
    }

    //Get target position
    public synchronized  int getTargetPosition(){
        return targetPosition;
    }

    /**
     * Indicates whether approaching target in RUN_TO_POSITION mode
     * Result will become false when: ticks are nearly at the target AND speed is very slow
     */
    public synchronized boolean isBusy(){
        int pos = getCurrentPosition();
        boolean atTarget = Math.abs(pos-targetPosition)/MOTOR_TYPE.TICKS_PER_ROTATION < MAX_ROT_OFFSET;
        boolean almostStopped = Math.abs(actualSpeed) / (COEFF_PROPORTIONATE * MOTOR_TYPE.TICKS_PER_ROTATION) < MAX_ROT_OFFSET;
        return mode == RunMode.RUN_TO_POSITION && Math.abs(power) > 0.0001 && (!atTarget || !almostStopped);
    }

    public synchronized void setZeroPowerBehavior(ZeroPowerBehavior beh) { zeroPowerBehavior = beh; }

    public synchronized ZeroPowerBehavior getZeroPowerBehavior() { return zeroPowerBehavior; }

    public MotorConfigurationType getMotorType(){
        return MOTOR_CONFIGURATION_TYPE;
    }

    public synchronized void setUpperPositionLimitEnabled(boolean upperLimitEnabled){
        this.upperPositionLimitEnabled = upperLimitEnabled;
    }

    public synchronized void setLowerPositionLimitEnabled(boolean lowerLimitEnabled){
        this.lowerPositionLimitEnabled = lowerLimitEnabled;
    }

    public synchronized void setUpperActualPositionLimit(double upperPositionLimit){
        this.upperActualPositionLimit = upperPositionLimit;
    }

    public synchronized void setLowerActualPositionLimit(double lowerPositionLimit){
        this.lowerActualPositionLimit = lowerPositionLimit;
    }

    public synchronized void setPositionLimitsEnabled(boolean limitsEnabled){
        this.upperPositionLimitEnabled = limitsEnabled;
        this.lowerPositionLimitEnabled = limitsEnabled;
    }

    public synchronized void setActualPositionLimits(double lowerPositionLimit, double upperPositionLimit){
        this.upperActualPositionLimit = upperPositionLimit;
        this.lowerActualPositionLimit = lowerPositionLimit;
    }

    public DcMotorController getController(){
        return controller;
    }

    public int getPortNumber(){
        return portNumber;
    }

}
