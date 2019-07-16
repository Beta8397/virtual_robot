package com.qualcomm.robotcore.hardware;

public class ServoImpl implements Servo{
    private double position;

    public synchronized void setPosition(double position) {
        this.position = Math.max(0, Math.min(1, position));
    }

    public synchronized double getPosition(){
        return position;
    }
}
