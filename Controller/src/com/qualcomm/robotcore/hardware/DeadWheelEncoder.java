package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.configuration.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DeadWheelEncoder implements DcMotorEx {

    public final MotorType MOTOR_TYPE;
    private final MotorConfigurationType MOTOR_CONFIGURATION_TYPE;

    RunMode mode = RunMode.RUN_WITHOUT_ENCODER;

    private Direction direction = Direction.FORWARD;

    //actual position of motor
    private double actualPosition = 0.0;

    //velocity of motor
    private double velocityRadiansPerSecond = 0.0;

    //position to use as baseline for encoder tick calculation
    private double encoderBasePosition = 0.0;

    private double power = 0.0;

    private int targetPosition = 0;

    private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;

    public DeadWheelEncoder(MotorType motorType) {
        MOTOR_TYPE = motorType;
        MOTOR_CONFIGURATION_TYPE = new MotorConfigurationType(motorType);
    }

    @Override
    public synchronized void setMode(RunMode mode) {
        this.mode = mode;
        power = 0.0;
        if (mode == RunMode.STOP_AND_RESET_ENCODER){
            encoderBasePosition = actualPosition;
        }
    }

    @Override
    public synchronized RunMode getMode() {
        return mode;
    }

    @Override
    public synchronized int getCurrentPosition() {
        int result = (int)Math.floor(actualPosition - encoderBasePosition);
        return direction == Direction.FORWARD && MOTOR_TYPE.REVERSED ||
                direction == Direction.REVERSE && !MOTOR_TYPE.REVERSED ? -result : result;
    }

    @Override
    public synchronized void setTargetPosition(int pos) { targetPosition = pos; }

    @Override
    public synchronized int getTargetPosition() { return targetPosition; }

    @Override
    public synchronized boolean isBusy() {
        return false;
    }

    @Override
    public synchronized void setDirection(Direction direction) { this.direction = direction; }

    @Override
    public synchronized Direction getDirection() { return direction; }

    @Override
    public synchronized void setPower(double power) {
        this.power = power;
    }

    @Override
    public synchronized double getPower() {
        return power;
    }

    /**
     * Update position of the deadwheel encoder
     * @param radians Incremental clockwise rotation of the dead wheel in radians
     */
    public synchronized void update( double radians, double milliseconds ){
        final double TWO_PI = 2.0 * Math.PI;
        double positionChange = radians * MOTOR_TYPE.TICKS_PER_ROTATION / TWO_PI;
        double tempVelocity = radians / milliseconds * 1000.0;
        if ((MOTOR_TYPE.REVERSED && direction == Direction.FORWARD) ||
                (!MOTOR_TYPE.REVERSED && direction == Direction.REVERSE)) {
            tempVelocity = -tempVelocity;
        }
        actualPosition += positionChange;
        velocityRadiansPerSecond = tempVelocity;
    }

    //For internal use only: for stopping and resetting motor between op mode runs
    public synchronized void stopAndReset(){
        power = 0.0;
        actualPosition = 0.0;
        encoderBasePosition = 0.0;
        direction = Direction.FORWARD;
    }

    public synchronized void setZeroPowerBehavior(ZeroPowerBehavior beh){ zeroPowerBehavior = beh; }
    public synchronized ZeroPowerBehavior getZeroPowerBehavior() { return zeroPowerBehavior; }


    @Override
    public void setMotorEnable() {
        throw new NotImplementedException("setMotorEnable method has not been implemented in DcMotorExImpl.");
    }

    @Override
    public void setMotorDisable() {
        throw new NotImplementedException("setMotorDisable method has not been implemented in DcMotorExImpl.");
    }

    @Override
    public boolean isMotorEnabled() {
        return true;
    }

    /**
     * Set desired velocity (in ticks per second)
     * @param angularRate  the desired ticks per second
     */
    @Override
    public synchronized void setVelocity(double angularRate) {
        setPower(angularRate/MOTOR_TYPE.MAX_TICKS_PER_SECOND);
    }

    /**
     * Set desired velocity in specified angular units
     * @param angularRate   the desired angular rate, in units per second
     * @param unit          the units in which angularRate is expressed
     *
     */
    @Override
    public synchronized void setVelocity(double angularRate, AngleUnit unit) {
        double unitsPerRotation = unit == AngleUnit.DEGREES? 360.0 : 2.0 * Math.PI;
        double ticksPerSecond = angularRate * MOTOR_TYPE.TICKS_PER_ROTATION / unitsPerRotation;
        setVelocity(ticksPerSecond);
    }

    /**
     * Get current velocity of motor in ticks per second
     * @return velocity in ticks per second
     */
    @Override
    public synchronized double getVelocity() {
        return velocityRadiansPerSecond  * MOTOR_TYPE.TICKS_PER_ROTATION / (2.0 * Math.PI);
    }

    @Override
    public synchronized double getVelocity(AngleUnit unit) {
        double result = velocityRadiansPerSecond;
        if (unit == AngleUnit.DEGREES) result *= 180.0 / Math.PI;
        return result;
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        throw new NotImplementedException("setPIDCoefficients method has not been implemented in DcMotorExImpl.");
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        throw new NotImplementedException("setPIDFCoefficients method has not been implemented in DcMotorExImpl.");
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        throw new NotImplementedException("setVelocityPIDFCoefficients method has not been implemented in DcMotorExImpl.");
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        throw new NotImplementedException("setPositionPIDFCoefficients method has not been implemented in DcMotorExImpl.");
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        throw new NotImplementedException("getPIDCoefficients method has not been implemented in DcMotorExImpl.");
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        throw new NotImplementedException("getPIDFCoefficients method has not been implemented in DcMotorExImpl.");
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        throw new NotImplementedException("setTargetPositionTolerance method has not been implemented in DcMotorExImpl.");
    }

    @Override
    public int getTargetPositionTolerance() {
        throw new NotImplementedException("getTargetPositionTolerance method has not been implemented in DcMotorExImpl.");
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return 0;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return 0;
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {

    }

    @Override
    public boolean isOverCurrent() {
        return false;
    }

    @Override
    public MotorConfigurationType getMotorType(){
        return MOTOR_CONFIGURATION_TYPE;
    }

}
