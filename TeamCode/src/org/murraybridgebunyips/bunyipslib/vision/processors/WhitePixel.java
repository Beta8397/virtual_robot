package org.murraybridgebunyips.bunyipslib.vision.processors;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class WhitePixel extends YCbCrColourThreshold {
    public static double LOWER_Y = 192.7;
    public static double LOWER_CB = 123.3;
    public static double LOWER_CR = 106.3;
    public static double UPPER_Y = 255;
    public static double UPPER_CB = 255;
    public static double UPPER_CR = 255;

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
}
