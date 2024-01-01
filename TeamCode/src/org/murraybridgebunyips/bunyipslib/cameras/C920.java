package org.murraybridgebunyips.bunyipslib.cameras;

/**
 * Represents a Logitech C920 camera intrinsic calibration.
 * This is also acceptable to use as a default for other cameras.
 */
public class C920 extends CameraType {
    public C920() {
        fx = 578.272;
        fy = 578.272;
        cx = 402.145;
        cy = 221.506;
    }
}
