package com.qualcomm.robotcore.hardware;

public class DeadWheelEncoder implements DcMotor {

    public final MotorType MOTOR_TYPE;

    RunMode mode = RunMode.RUN_WITHOUT_ENCODER;

    private Direction direction = Direction.FORWARD;

    //actual position of motor
    private double actualPosition = 0.0;

    //position to use as baseline for encoder tick calculation
    private double encoderBasePosition = 0.0;

    private double power = 0.0;

    private int targetPosition = 0;

    private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;

    public DeadWheelEncoder(MotorType motorType) { MOTOR_TYPE = motorType; }

    @Override
    public void setMode(RunMode mode) {
        this.mode = mode;
        power = 0.0;
        if (mode == RunMode.STOP_AND_RESET_ENCODER){
            encoderBasePosition = actualPosition;
        }
    }

    @Override
    public RunMode getMode() {
        return mode;
    }

    @Override
    public int getCurrentPosition() {
        int result = (int)Math.floor(actualPosition - encoderBasePosition);
        return direction == Direction.FORWARD && MOTOR_TYPE.REVERSED ||
                direction == Direction.REVERSE && !MOTOR_TYPE.REVERSED ? -result : result;
    }

    @Override
    public void setTargetPosition(int pos) { targetPosition = pos; }

    @Override
    public int getTargetPosition() { return targetPosition; }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public void setDirection(Direction direction) { this.direction = direction; }

    @Override
    public Direction getDirection() { return direction; }

    @Override
    public void setPower(double power) {
        this.power = power;
    }

    @Override
    public double getPower() {
        return power;
    }

    /**
     * Update position of the deadwheel encoder
     * @param radians Incremental clockwise rotation of the dead wheel in radians
     */
    public void update( double radians ){
        final double TWO_PI = 2.0 * Math.PI;
        double positionChange = radians * MOTOR_TYPE.TICKS_PER_ROTATION / TWO_PI;
        if ((MOTOR_TYPE.REVERSED && direction == Direction.FORWARD) ||
                (!MOTOR_TYPE.REVERSED && direction == Direction.REVERSE)) positionChange = -positionChange;
        actualPosition += positionChange;
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

}
