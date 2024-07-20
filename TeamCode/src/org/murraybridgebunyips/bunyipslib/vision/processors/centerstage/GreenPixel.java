package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;
import org.opencv.core.Scalar;

/**
 * Green pixel processor.
 * These values may not be tuned for your specific camera, lighting, or field conditions, and are tuned
 * based on our own testing. You may need to adjust these values to get the best results for your own robot.
 */
@Config
public class GreenPixel extends ColourThreshold {
    /**
     * Lower clamp for YCrCb
     */
    public static Scalar LOWER_YCBCR = new Scalar(145, 0, 0);
    /**
     * Upper clamp for YCrCb
     */
    public static Scalar UPPER_YCBCR = new Scalar(255, 120, 255);
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
    protected Scalar setLower() {
        return LOWER_YCBCR;
    }

    @Override
    protected Scalar setUpper() {
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
