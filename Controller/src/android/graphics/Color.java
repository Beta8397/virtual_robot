package android.graphics;

import com.sun.javafx.util.Utils;

/**
 * Provides an RGBtoHSV method with signature identical to that found in the android.graphics.Color class
 */
public class Color {

    /**
     * Given inputs of red, green, and blue channels, writes the hue, sat, and value to the hsv output array
     * @param red   red (0..255)
     * @param green green (0..255)
     * @param blue  blue (0..255)
     * @param hsv   Output array for return of hue (0..360), sat (0..1), and value (0..1)
     */
    public static void RGBtoHSV(int red, int green, int blue, float[] hsv){
        double[] temp = Utils.RGBtoHSB(red, green, blue);
        hsv[0] = (float)temp[0];
        hsv[1] = (float)temp[1];
        hsv[2] = (float)temp[2];
    }
}
