package android.graphics;


/**
 * Provides an RGBToHSV method with signature identical to that found in the android.graphics.Color class
 */
public class Color {
    public static int BLUE = argb(0,0,0,255);
    public static int GREEN = argb(0, 0, 255, 0);

    public static int RED = argb(0, 255, 0, 0);

    /**
     * Given inputs of red, green, and blue channels, writes the hue, sat, and value to the hsv output array
     * @param red   red (0..255)
     * @param green green (0..255)
     * @param blue  blue (0..255)
     * @param hsv   Output array for return of hue (0..360), sat (0..1), and value (0..1)
     */
    public static void RGBToHSV(int red, int green, int blue, float[] hsv){
        javafx.scene.paint.Color color = javafx.scene.paint.Color.rgb(red, green, blue);
        hsv[0] = (float)color.getHue();
        hsv[1] = (float)color.getSaturation();
        hsv[2] = (float)color.getBrightness();
    }

    public static int HSVToColor(float[] hsv){
        javafx.scene.paint.Color color = javafx.scene.paint.Color.hsb(hsv[0], hsv[1], hsv[2]);
        int red = (int)Math.floor(256.0*color.getRed());
        if (red == 256) red = 255;
        int green = (int)Math.floor(256.0*color.getGreen());
        if (green == 256) green = 255;
        int blue = (int)Math.floor(256.0*color.getBlue());
        if (blue == 256) blue = 255;
        int colorInt = (red << 16 ) | (green << 8) | blue;
        return colorInt;
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
