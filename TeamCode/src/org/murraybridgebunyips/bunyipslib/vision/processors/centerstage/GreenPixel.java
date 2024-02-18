package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.vision.processors.YCbCrColourThreshold;
import org.opencv.core.Scalar;

@Config
public class GreenPixel extends YCbCrColourThreshold {
    public static double LOWER_Y = 80.0;
    public static double LOWER_CB = 0.0;
    public static double LOWER_CR = 70.0;
    public static double UPPER_Y = 255.0;
    public static double UPPER_CB = 100.0;
    public static double UPPER_CR = 255.0;
    public static boolean SHOW_MASKED_INPUT = true;

    @Override
    public String getName() {
        return "greenpixel";
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
        return 0xFF00FF00;
    }

    @Override
    public boolean showMaskedInput() {
        return SHOW_MASKED_INPUT;
    }
}
