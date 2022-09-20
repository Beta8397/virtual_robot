package android.graphics;

import com.sun.javafx.util.Utils;

/**
 * Provides an RGBToHSV method with signature identical to that found in the android.graphics.Color class
 */
public class Color {

    /**
     * Given inputs of red, green, and blue channels, writes the hue, sat, and value to the hsv output array
     * @param red   red (0..255)
     * @param green green (0..255)
     * @param blue  blue (0..255)
     * @param hsv   Output array for return of hue (0..360), sat (0..1), and value (0..1)
     */
    public static void RGBToHSV(int red, int green, int blue, float[] hsv){
        double[] temp = Utils.RGBtoHSB(red, green, blue);
        hsv[0] = (float)temp[0];
        hsv[1] = (float)temp[1];
        hsv[2] = (float)temp[2];
    }

    public static int HSVToColor(float[] hsv){
        double[] rgb = Utils.HSBtoRGB(hsv[0], hsv[1], hsv[2]);
        int red = (int)Math.floor(256.0*rgb[0]);
        if (red == 256) red = 255;
        int green = (int)Math.floor(256.0*rgb[1]);
        if (green == 256) green = 255;
        int blue = (int)Math.floor(256.0*rgb[2]);
        if (blue == 256) blue = 255;
        int color = (red << 16 ) | (green << 8) | blue;
        return color;
    }

    public static int HSVToColor(int alpha, float[] hsv){
        int result = HSVToColor(hsv);
        result = result | ( alpha << 24 );
        return result;
    }

    public static int alpha(int color){
        return (color >> 24) & 0xFF;
    }

    public static int red(int color){
        return (color >> 16) & 0xFF;
    }

    public static int green(int color){
        return (color >> 8) & 0xFF;
    }

    public static int blue(int color){
        return color & 0xFF;
    }

    public static int argb(
            int alpha,
            int red,
            int green,
            int blue) {
        return (alpha << 24) | (red << 16) | (green << 8) | blue;
    }
}
