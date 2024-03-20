package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;
import org.opencv.core.Scalar;

@Config
public class RedTeamProp extends ColourThreshold {
    public static Scalar lower = new Scalar(0, 190, 0);
    public static Scalar upper = new Scalar(200, 255, 102);
    public static double min = 5;
    public static double max = 100;

    /**
     * Defines a new colour thresholding processor for a specific colour space, which your
     * lower and upper scalars will be based on.
     */
    public RedTeamProp() {
        super(ColourSpace.YCrCb);
    }

    @Override
    public String getName() {
        return "redteamprop";
    }

    @Override
    public double getContourAreaMinPercent() {
        return min;
    }

    @Override
    public double getContourAreaMaxPercent() {
        return max;
    }

    @Override
    public Scalar getLower() {
        return lower;
    }

    @Override
    public Scalar getUpper() {
        return upper;
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
