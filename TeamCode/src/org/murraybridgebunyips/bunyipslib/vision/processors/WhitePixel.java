package org.murraybridgebunyips.bunyipslib.vision.processors;

import org.opencv.core.Scalar;

public class WhitePixel extends YCbCrColourThreshold {
    @Override
    public String getName() {
        return "whitepixel";
    }

    @Override
    public Scalar getLower() {
        return new Scalar(192.7, 123.3, 106.3);
    }

    @Override
    public Scalar getUpper() {
        return new Scalar(255, 255, 255);
    }
}
