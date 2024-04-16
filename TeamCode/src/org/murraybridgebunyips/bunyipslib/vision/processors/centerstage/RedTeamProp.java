package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;
import org.opencv.core.Scalar;

/**
 * Processor for the red team prop.
 *
 * @author Lucas Bubner, 2024
 */
@Config
public class RedTeamProp extends ColourThreshold {
    /**
     * The lower YCrCb bounds for the red team prop.
     */
    public static Scalar LOWER = new Scalar(0, 190, 0);
    /**
     * The upper YCrCb bounds for the red team prop.
     */
    public static Scalar UPPER = new Scalar(200, 255, 102);
    /**
     * The minimum contour area percentages for the red team prop.
     */
    public static double MIN = 5;
    /**
     * The maximum contour area percentages for the red team prop.
     */
    public static double MAX = 100;

    /**
     * Defines a new colour thresholding processor for a specific colour space, which your
     * lower and upper scalars will be based on.
     */
    public RedTeamProp() {
        super(ColourSpace.YCrCb);
    }

    @NonNull
    @Override
    public String toString() {
        return "redteamprop";
    }

    @Override
    public double getContourAreaMinPercent() {
        return MIN;
    }

    @Override
    public double getContourAreaMaxPercent() {
        return MAX;
    }

    @Override
    protected Scalar setLower() {
        return LOWER;
    }

    @Override
    protected Scalar setUpper() {
        return UPPER;
    }

    @Override
    public int getBoxColour() {
        return 0xffff0000;
    }

    @Override
    public boolean showMaskedInput() {
        return true;
    }
}
