package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.vision.processors.YCbCrColourThreshold;
import org.opencv.core.Scalar;

@Config
public class PurplePixel extends YCbCrColourThreshold {
    public static double LOWER_Y = 89.3;
    public static double LOWER_CB = 0.0;
    public static double LOWER_CR = 131.8;
    public static double UPPER_Y = 255.0;
    public static double UPPER_CB = 255.0;
    public static double UPPER_CR = 255.0;
    public static boolean SHOW_MASKED_INPUT = true;

    @Override
    public String getName() {
        return "purplepixel";
    }

    @Override
    public Scalar getLower() {
        return new Scalar(LOWER_Y, LOWER_CB, LOWER_CR);
    }

    @Override
    public Scalar getUpper() {
        return new Scalar(UPPER_Y, UPPER_CB, UPPER_CR);
    }

    @Override
    public int getBoxColour() {
        return 0xFF800080;
    }

    @Override
    public boolean showMaskedInput() {
        return SHOW_MASKED_INPUT;
    }
}
