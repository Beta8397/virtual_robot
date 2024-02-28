package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;
import org.opencv.core.Scalar;

@Config
public class YellowPixel extends ColourThreshold {
    public static Scalar LOWER_YCBCR = new Scalar(0.0, 150.0, 0.0);
    public static Scalar UPPER_YCBCR = new Scalar(255.0, 255.0, 82.2);
    public static double MIN_AREA = DEFAULT_MIN_AREA;
    public static double MAX_AREA = DEFAULT_MAX_AREA;
    public static boolean SHOW_MASKED_INPUT = true;

    public YellowPixel() {
        super(ColourSpace.YCrCb);
    }

    @Override
    public String getName() {
        return "yellowpixel";
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
        return 0xFFFFFF00;
    }

    @Override
    public boolean showMaskedInput() {
        return SHOW_MASKED_INPUT;
    }
}
