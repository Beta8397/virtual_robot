package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;
import org.opencv.core.Scalar;

/**
 * Green pixel processor
 */
@Config
public class GreenPixel extends ColourThreshold {
    /**
     * Lower clamp for YCrCb
     */
    public static Scalar LOWER_YCBCR = new Scalar(80, 0, 70);
    /**
     * Upper clamp for YCrCb
     */
    public static Scalar UPPER_YCBCR = new Scalar(255, 100, 255);
    /**
     * Default minimum area for the contour
     */
    public static double MIN_AREA = DEFAULT_MIN_AREA;
    /**
     * Default maximum area for the contour
     */
    public static double MAX_AREA = DEFAULT_MAX_AREA;
    /**
     * Whether to show the masked input
     */
    public static boolean SHOW_MASKED_INPUT = true;

    /**
     * Using YCrCb colour space
     */
    public GreenPixel() {
        super(ColourSpace.YCrCb);
    }

    @NonNull
    @Override
    public String toString() {
        return "greenpixel";
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
        return 0xFF00FF00;
    }

    @Override
    public boolean showMaskedInput() {
        return SHOW_MASKED_INPUT;
    }
}
