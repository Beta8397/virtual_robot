package com.qualcomm.robotcore.hardware;

public class PassiveColorSensorImpl implements ColorSensor {

    private int red = 0;
    private int green = 0;
    private int blue = 0;
    private int alpha = 0;

    /**
     * Internal Use Only. Update the color sensor using provided values of R, G, B
     */
    public synchronized void updateColor(int R, int G, int B){
        red = R;
        green = G;
        blue = B;
        alpha = Math.max(red, Math.max(green, blue));
    }

    @Override
    public synchronized int red() {
        return red;
    }

    @Override
    public synchronized int green() {
        return green;
    }

    @Override
    public synchronized int blue() {
        return blue;
    }

    @Override
    public synchronized int alpha() {
        return alpha;
    }
}
