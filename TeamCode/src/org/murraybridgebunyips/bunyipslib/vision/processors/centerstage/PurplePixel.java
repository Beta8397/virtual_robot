package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;
import org.opencv.core.Scalar;

/**
 * Purple pixel processor
 */
@Config
public class PurplePixel extends ColourThreshold {
    /**
     * Lower bounds for the YCrCb colour space.
     */
    public static Scalar LOWER_YCBCR = new Scalar(150, 0.0, 145.8);
    /**
     * Upper bounds for the YCrCb colour space.
     */
    public static Scalar UPPER_YCBCR = new Scalar(255.0, 255.0, 255.0);
    /**
     * Default minimum area for the contour.
     */
    public static double MIN_AREA = DEFAULT_MIN_AREA;
    /**
     * Default maximum area for the contour.
     */
    public static double MAX_AREA = DEFAULT_MAX_AREA;
    /**
     * Whether to show the masked input on the screen.
     */
    public static boolean SHOW_MASKED_INPUT = true;

    /**
     * Using the YCrCb colour space.
     */
    public PurplePixel() {
        super(ColourSpace.YCrCb);
    }

    @NonNull
    @Override
    public String toString() {
        return "purplepixel";
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
    protected Scalar setLower() {
        return LOWER_YCBCR;
    }

    @Override
    protected Scalar setUpper() {
        return UPPER_YCBCR;
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
