package org.murraybridgebunyips.bunyipslib.cameras;

/**
 * Represents a camera intrinsic calibration for use with Vision processing.
 */
public abstract class CameraType {
    protected double fx;
    protected double fy;
    protected double cx;
    protected double cy;

    public double getFx() {
        return fx;
    }

    public double getFy() {
        return fy;
    }

    public double getCx() {
        return cx;
    }

    public double getCy() {
        return cy;
    }
}
