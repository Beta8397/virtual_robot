package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import org.murraybridgebunyips.bunyipslib.vision.processors.YCbCrColourThreshold;

/**
 * Utility construction for pixel detectors.
 */
public class Pixels {
    private Pixels() {
    }

    public static YCbCrColourThreshold[] createProcessors() {
        return new YCbCrColourThreshold[]{
                new WhitePixel(),
                new PurplePixel(),
                new YellowPixel(),
                new GreenPixel()
        };
    }
}
