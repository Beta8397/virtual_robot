package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.configuration.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

/**
 * Implementation of the DcMotor interface.
 */
public class DcMotorDynImpl implements DcMotor {
    public final MotorType MOTOR_TYPE;
    public final MotorConfigurationType MOTOR_CONFIGURATION_TYPE;
    public final DcMotorControllerImpl controller;
    public final int portNumber;

    // Proportionate coefficient for RUN_TO_POSITION mode
    protected final double POS_COEFF_PROPORTIONATE = 0.001;

    // Proportionate coefficient for RUN_USING_ENCODER mode
    protected final double SPEED_COEFF_PROPORTIONATE = 0.5;


    // Rotation offset (in rotations) for isBusy
    protected final double MAX_ROT_OFFSET = 0.005;

    //Target position for RUN_TO_POSITION mode
    private int targetPosition = 0;
    private boolean targetPositionIsSet = false;

    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private Direction direction = Direction.FORWARD;

    /* power is the requested power, clipped to the -1 to +1 range.
     * This means different things, depending on the run mode:
     *
     *    RUN_WITHOUT_ENCODER: equal to plus or minus the raw power
     *    RUN_USING_ENCODER: plus or minus the requested speed as a fraction of max achievable speed
     *    RUN_TO_POSITION: plus or minus max requested speed as a fraction of max achievable speed
     */
    private double power = 0.0;

    // rawpower is the actual PWM fraction being supplied to the motor (positive -> CW wheel rotation)
    private double rawPower = 0.0;

    /*
     * actualSpeed is current motor "actual speed", normalized to the -1 to +1 range. This refers
     * to the rate at which the output shaft is rotating in the CLOCKWISE direction (when viewed from
     * the shaft end). This will be equal in magnitude to speed, but may differ in sign, depending
     * upon the values of direction and MOTOR_TYPE.reversed.
     */
    private double actualSpeed = 0.0;

    //actual position of motor, in ticks
    private double actualPosition = 0.0;

    //position to use as baseline for encoder tick calculation
    private double encoderBasePosition = 0.0;

    private final boolean supportsError = false;
    private final boolean supportsInertia = false;

    private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;
    // Proportionate coefficient for zero-power slowing
    protected double ZERO_POWER_COEFF_PROPORTIONATE = 1.0;

    /**
     * For internal use only.
     * @param motorType
     */
    public DcMotorDynImpl(MotorType motorType, DcMotorControllerImpl controller, int portNumber){
        MOTOR_TYPE = motorType;
        MOTOR_CONFIGURATION_TYPE = new MotorConfigurationType(motorType);
        this.controller = controller;
        this.portNumber = portNumber;
        controller.setMotor(portNumber, this);
    }


    /**
     * Set mode of operation
     * @param mode
     */
    public synchronized void setMode(RunMode mode){
        this.mode = mode;
        power = 0.0;
        rawPower = 0;
        actualSpeed = 0;
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
     * Get raw power
     */
    public synchronized double getRawPower(){ return rawPower;}

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


    /*
     * For internal use only.
     * New version of the update method for dynamic robot drivetrain model.
     * Updates motor speed and position given current speed from physics engine,
     * updates rawPower, then computes torque output by motor for next physics cycle.
     * @param milliseconds number of milliseconds since last update
     * @param newActualSpeed (new actual motor speed in -1 to +1 range)
     * @return motor torque in Newton-inches
     */
    public synchronized double update(double milliseconds, double newActualSpeed){

        if (mode == RunMode.STOP_AND_RESET_ENCODER){
            rawPower = 0;
            actualSpeed = 0;
            return 0;
        }

        boolean rev = direction == Direction.FORWARD && MOTOR_TYPE.REVERSED
                || direction == Direction.REVERSE && !MOTOR_TYPE.REVERSED;


        actualPosition += 0.5 * (actualSpeed + newActualSpeed) * MOTOR_TYPE.MAX_TICKS_PER_SECOND * milliseconds / 1000.0;
        actualSpeed = newActualSpeed;

        if (mode == RunMode.RUN_TO_POSITION){
            double actualTargetPosition = rev? -targetPosition : targetPosition;
            double tentativeRawPower = (double)(actualTargetPosition - (actualPosition-encoderBasePosition))
                    * POS_COEFF_PROPORTIONATE;
            double absPower = Math.abs(power);
            if (Math.abs(tentativeRawPower) > absPower){
                tentativeRawPower = absPower * Math.signum(tentativeRawPower);
            }
            rawPower = tentativeRawPower;
        } else if (mode == RunMode.RUN_USING_ENCODER){
            double requestedActualSpeed = rev? -power : power;
            requestedActualSpeed *= MOTOR_TYPE.ACHIEVABLE_MAX_RPM_FRACTION;
            if (power == 0){
                rawPower = 0;
            } else {
                rawPower = requestedActualSpeed + (requestedActualSpeed - actualSpeed) * SPEED_COEFF_PROPORTIONATE;
            }
        } else {
            rawPower = rev? -power : power;
        }

        rawPower = Range.clip(rawPower, -1, 1);

        double torque;
        if (power == 0) {
            torque = -actualSpeed * ZERO_POWER_COEFF_PROPORTIONATE;
        } else if (rawPower >= 0){
            torque = rawPower * MOTOR_TYPE.MAX_TORQUE * (1 - actualSpeed / Math.abs(rawPower));
            torque = Range.clip(torque, 0, MOTOR_TYPE.MAX_TORQUE);
        } else {
            torque = rawPower * MOTOR_TYPE.MAX_TORQUE * (1 + actualSpeed / Math.abs(rawPower));
            torque = Range.clip(torque, -MOTOR_TYPE.MAX_TORQUE, 0);
        }

        return torque;

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
        return;
    }

    /**
     * For internal use only.
     * @param sysErrFrac
     */
    public synchronized void setSystematicErrorFrac(double sysErrFrac) {
        return;
    }

    /**
     * For internal use only.
     * @param in
     */
    public synchronized void setInertia(double in){
        return;
    }

    //For internal use only: for stopping and resetting motor between op mode runs
    public synchronized void stopAndReset(){
        power = 0.0;
        rawPower = 0;
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
        boolean almostStopped = Math.abs(actualSpeed) / (POS_COEFF_PROPORTIONATE * MOTOR_TYPE.TICKS_PER_ROTATION) < MAX_ROT_OFFSET;
        return mode == RunMode.RUN_TO_POSITION && Math.abs(power) > 0.0001 && (!atTarget || !almostStopped);
    }

    public synchronized void setZeroPowerBehavior(ZeroPowerBehavior beh) {
        zeroPowerBehavior = beh;
        ZERO_POWER_COEFF_PROPORTIONATE = beh == ZeroPowerBehavior.BRAKE? 1.0 : 0.1;
    }

    public synchronized ZeroPowerBehavior getZeroPowerBehavior() { return zeroPowerBehavior; }

    public MotorConfigurationType getMotorType(){
        return MOTOR_CONFIGURATION_TYPE;
    }

    public synchronized void setUpperPositionLimitEnabled(boolean upperLimitEnabled){
        return;
    }

    public synchronized void setLowerPositionLimitEnabled(boolean lowerLimitEnabled){
        return;
    }

    public synchronized void setUpperActualPositionLimit(double upperPositionLimit){
        return;
    }

    public synchronized void setLowerActualPositionLimit(double lowerPositionLimit){
        return;
    }

    public synchronized void setPositionLimitsEnabled(boolean limitsEnabled){
        return;
    }

    public synchronized void setActualPositionLimits(double lowerPositionLimit, double upperPositionLimit){
        return;
    }

    public DcMotorController getController(){
        return controller;
    }

    public int getPortNumber(){
        return portNumber;
    }

}
