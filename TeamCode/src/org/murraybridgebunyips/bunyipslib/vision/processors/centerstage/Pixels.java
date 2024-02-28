package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;

/**
 * Utility construction for pixel detectors.
 */
public class Pixels {
    private Pixels() {
    }

    /**
     * Create all the pixel processors.
     *
     * @return An array of all the pixel processors - White Pixel, Purple Pixel, Yellow Pixel, and Green Pixel
     */
    public static ColourThreshold[] createProcessors() {
        return new ColourThreshold[]{
                new WhitePixel(),
                new PurplePixel(),
                new YellowPixel(),
                new GreenPixel()
        };
    }
}
