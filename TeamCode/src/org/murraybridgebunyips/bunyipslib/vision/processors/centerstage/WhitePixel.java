package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;
import org.opencv.core.Scalar;

@Config
public class WhitePixel extends ColourThreshold {
    public static Scalar LOWER_YCBCR = new Scalar(192.7, 120.0, 106.3);
    public static Scalar UPPER_YCBCR = new Scalar(255.0, 129.0, 255.0);
    public static double MIN_AREA = DEFAULT_MIN_AREA;
    public static double MAX_AREA = DEFAULT_MAX_AREA;
    public static boolean SHOW_MASKED_INPUT = true;

    public WhitePixel() {
        super(ColourSpace.YCrCb);
    }

    @Override
    public String getName() {
        return "whitepixel";
    }

    @Override
    public double getContourAreaMinPercent() {
        return MIN_AREA;
    }

    @Override
    public double getContourAreaMaxPercent() {
        return MAX_AREA;
    }

    @Override
    public Scalar getLower() {
        return LOWER_YCBCR;
    }

    @Override
    public Scalar getUpper() {
        return UPPER_YCBCR;
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
