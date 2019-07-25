package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * An abridged version of the FTC ColorSensor interface.
 */
public interface ColorSensor extends HardwareDevice {

    /**
     * @return Red channel value, 0..255
     */
    public int red();

    /**
     * @return Green channel value, 0..255
     */
    public int green();

    /**
     * @return Blue channel value, 0..255
     */
    public int blue();

}
