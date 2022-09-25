package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.configuration.MotorType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * A very limited implementation of DcMotorEx.
 *
 * This is included to support existing Team Code that uses Bulk Cache Reads.
 */

public class DcMotorExImpl extends DcMotorImpl implements DcMotorEx{

    /**
     * For internal use only.
     * @param motorType
     */
    public DcMotorExImpl(MotorType motorType){
        super(motorType);
    }

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
        return getSpeed() * MOTOR_TYPE.MAX_TICKS_PER_SECOND;
    }

    @Override
    public synchronized double getVelocity(AngleUnit unit) {
        double unitsPerRotation = unit == AngleUnit.DEGREES? 360.0 : 2.0 * Math.PI;
        return getVelocity() * unitsPerRotation / MOTOR_TYPE.TICKS_PER_ROTATION;
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        // allows use of code that calls this but doesn't do anything
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
        return new PIDFCoefficients(COEFF_PROPORTIONATE, 0, 0, 0);
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
}
