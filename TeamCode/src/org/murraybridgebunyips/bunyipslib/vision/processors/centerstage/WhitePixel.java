package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.vision.processors.YCbCrColourThreshold;
import org.opencv.core.Scalar;

@Config
public class WhitePixel extends YCbCrColourThreshold {
    public static double LOWER_Y = 192.7;
    public static double LOWER_CB = 120.0;
    public static double LOWER_CR = 106.3;
    public static double UPPER_Y = 255;
    public static double UPPER_CB = 129;
    public static double UPPER_CR = 255;
    public static boolean SHOW_MASKED_INPUT = true;

    @Override
    public String getName() {
        return "whitepixel";
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
        return 0xFFFFFFFF;
    }

    @Override
    public boolean showMaskedInput() {
        return SHOW_MASKED_INPUT;
    }
}
