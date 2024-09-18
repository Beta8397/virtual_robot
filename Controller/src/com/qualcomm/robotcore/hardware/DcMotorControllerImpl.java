package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class DcMotorControllerImpl implements DcMotorController{

    DcMotor[] motors = new DcMotor[8];

    public void setMotor(int n, DcMotor motor){
        motors[n] = motor;
    }

    public DcMotorControllerImpl(){
        for (int n = 0; n<motors.length; n++){
            motors[n] = null;
        }
    }

    @Override
    public void setMotorType(int motor, MotorConfigurationType motorType) {
        motors[motor].setMotorType(motorType);
    }

    @Override
    public MotorConfigurationType getMotorType(int motor) {
        return motors[motor].getMotorType();
    }

    @Override
    public void setMotorMode(int motor, DcMotor.RunMode mode) {
        motors[motor].setMode(mode);
    }

    @Override
    public DcMotor.RunMode getMotorMode(int motor) {
        return motors[motor].getMode();
    }

    @Override
    public void setMotorPower(int motor, double power) {
        motors[motor].setPower(power);
    }

    @Override
    public double getMotorPower(int motor) {
        return motors[motor].getPower();
    }

    @Override
    public boolean isBusy(int motor) {
        return motors[motor].isBusy();
    }

    @Override
    public void setMotorZeroPowerBehavior(int motor, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motors[motor].setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public DcMotor.ZeroPowerBehavior getMotorZeroPowerBehavior(int motor) {
        return motors[motor].getZeroPowerBehavior();
    }

    @Override
    public boolean getMotorPowerFloat(int motor) {
        return motors[motor].getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.FLOAT;
    }

    @Override
    public void setMotorTargetPosition(int motor, int position) {
        motors[motor].setTargetPosition(position);
    }

    @Override
    public int getMotorTargetPosition(int motor) {
        return motors[motor].getTargetPosition();
    }

    @Override
    public int getMotorCurrentPosition(int motor) {
        return motors[motor].getCurrentPosition();
    }

    @Override
    public void resetDeviceConfigurationForOpMode(int motor) {
        motors[motor].resetDeviceConfigurationForOpMode();
    }
}
