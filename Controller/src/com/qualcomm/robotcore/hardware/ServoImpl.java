package com.qualcomm.robotcore.hardware;

/**
 * Implementation of the Servo interface.
 */
public class ServoImpl implements Servo{
    private double position;

    /**
     * Set position of servo.
     * @param position Must be between 0 and 1
     */
    public synchronized void setPosition(double position) {
        this.position = Math.max(0, Math.min(1, position));
    }

    /**
     * Get position of servo.
     * @return
     */
    public synchronized double getPosition(){
        return position;
    }
}
