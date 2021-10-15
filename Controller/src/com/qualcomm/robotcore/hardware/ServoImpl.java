package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.util.Range;

/**
 * Implementation of the Servo interface.
 */
public class ServoImpl implements Servo{

    private double position;

    protected Direction       direction        = Direction.FORWARD;
    protected double          limitPositionMin = MIN_POSITION;
    protected double          limitPositionMax = MAX_POSITION;

    /**
     * Set Direction of servo
     */
    public void setDirection(Direction direction) { this.direction = direction; }

    /**
     * Get Direction of servo
     */
    public Direction getDirection() { return this.direction; }

    /**
     * Scale the range of the servo
     */
    public void scaleRange(double min, double max){
        min = Range.clip(min, MIN_POSITION, MAX_POSITION);
        max = Range.clip(max, MIN_POSITION, MAX_POSITION);

        if (min >= max) {
            throw new IllegalArgumentException("min must be less than max");
        }

        limitPositionMin = min;
        limitPositionMax = max;
    }

    /**
     * Set position of servo.
     * @param pos Must be between 0 and 1
     */
    public synchronized void setPosition(double pos) {
        pos = Range.clip(pos, MIN_POSITION, MAX_POSITION);
        if (direction == Direction.REVERSE) pos = reverse(pos);
        double scaled = Range.scale(pos, MIN_POSITION, MAX_POSITION, limitPositionMin, limitPositionMax);
        this.position = scaled;
    }

    /**
     * Get position of servo.
     * @return
     */
    public synchronized double getPosition(){
        double pos = this.position;
        if (direction == Direction.REVERSE) pos = reverse(pos);
        double scaled = Range.scale(pos, limitPositionMin, limitPositionMax, MIN_POSITION, MAX_POSITION);
        return Range.clip(scaled, MIN_POSITION, MAX_POSITION);
    }

    /**
     * Get Internal Position -- FOR INTERNAL USE ONLY
     * @return
     */
    public synchronized double getInternalPosition(){ return this.position; }

    /**
     * Set the Internal Position of the Servo -- FOR INTERNAL USE ONLY
     * @param internalPos
     */
    public synchronized void setInternalPosition(double internalPos){
        this.position = Range.clip(internalPos, MIN_POSITION, MAX_POSITION);
    }


    private double reverse(double position) {
        return MAX_POSITION - position + MIN_POSITION;
    }

}
