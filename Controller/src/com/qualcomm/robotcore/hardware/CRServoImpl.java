package com.qualcomm.robotcore.hardware;

public class CRServoImpl implements CRServo {

    private DcMotorSimple.Direction direction = Direction.FORWARD;
    private double power = 0.0;
    private double positionDegrees = 0.0;
    public final double MAX_DEGREES_PER_SECOND;

    public CRServoImpl(double maxDegreesPerSecond){
        this.MAX_DEGREES_PER_SECOND = maxDegreesPerSecond;
    }

    @Override
    public synchronized void setDirection(DcMotorSimple.Direction direction){
        this.direction = direction;
    }

    @Override
    public synchronized DcMotorSimple.Direction getDirection(){ return this.direction; }

    @Override
    public synchronized void setPower(double power){
        if (power < -1.0) this.power = -1.0;
        else if (power > 1.0) this.power = 1.0;
        else this.power = power;
    }

    @Override
    public synchronized double getPower(){ return this.power; }

    public synchronized double updatePositionDegrees(double millis){
        double deltaPos = power * MAX_DEGREES_PER_SECOND * millis / 1000.0;
        positionDegrees += deltaPos;
        return deltaPos;
    }

    public synchronized double getPositionDegrees(){
        return positionDegrees;
    }


}
