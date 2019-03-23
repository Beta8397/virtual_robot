package virtual_robot.util.color;

import com.sun.javafx.util.Utils;

public class Color {
    public static void RGBtoHSV(int red, int green, int blue, float[] hsv){
        double[] temp = Utils.RGBtoHSB(red, green, blue);
        hsv[0] = (float)temp[0];
        hsv[1] = (float)temp[1];
        hsv[2] = (float)temp[2];
    }
}
