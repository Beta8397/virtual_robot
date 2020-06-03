package com.qualcomm.robotcore.hardware;

/**
 * Implementation of the CRServo interface.
 */
public class CRServoImpl implements CRServo {

    private DcMotorSimple.Direction direction = Direction.FORWARD;
    private double power = 0.0;
    private double positionDegrees = 0.0;
    public final double MAX_DEGREES_PER_SECOND;

    /**
     * For internal use only
     */
    public CRServoImpl(double maxDegreesPerSecond){
        this.MAX_DEGREES_PER_SECOND = maxDegreesPerSecond;
    }

    /**
     * Set the logical direction of the CRServo
     * @param direction the direction to set for this motor
     */
    @Override
    public synchronized void setDirection(DcMotorSimple.Direction direction){
        this.direction = direction;
    }

    /**
     * Get the logical direction of the CRServo
     * @return
     */
    @Override
    public synchronized DcMotorSimple.Direction getDirection(){ return this.direction; }

    /**
     * Set power
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     */
    @Override
    public synchronized void setPower(double power){
        if (power < -1.0) this.power = -1.0;
        else if (power > 1.0) this.power = 1.0;
        else this.power = power;
    }

    /**
     * Get power
     * @return
     */
    @Override
    public synchronized double getPower(){ return this.power; }

    /**
     * For internal use only.
     * @param millis
     * @return
     */
    public synchronized double updatePositionDegrees(double millis){
        double deltaPos = power * MAX_DEGREES_PER_SECOND * millis / 1000.0;
        positionDegrees += deltaPos;
        return deltaPos;
    }


    /**
     * For internal use only.
     * @return
     */
    public synchronized double getPositionDegrees(){
        return positionDegrees;
    }


}
