package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;
import org.opencv.core.Scalar;

/**
 * White pixel processor
 */
@Config
public class WhitePixel extends ColourThreshold {
    /**
     * Lower bounds for YCrCb
     */
    public static Scalar LOWER_YCBCR = new Scalar(192.7, 120.0, 106.3);
    /**
     * Upper bounds for YCrCb
     */
    public static Scalar UPPER_YCBCR = new Scalar(255.0, 129.0, 255.0);
    /**
     * Default min area for detections.
     */
    public static double MIN_AREA = DEFAULT_MIN_AREA;
    /**
     * Default max area for detections.
     */
    public static double MAX_AREA = DEFAULT_MAX_AREA;
    /**
     * Whether to show the masked input on the screen.
     */
    public static boolean SHOW_MASKED_INPUT = true;

    /**
     * Using YCrCb colour space.
     */
    public WhitePixel() {
        super(ColourSpace.YCrCb);
    }

    @NonNull
    @Override
    public String toString() {
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
