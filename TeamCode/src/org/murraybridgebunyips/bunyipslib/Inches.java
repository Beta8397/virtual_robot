package org.murraybridgebunyips.bunyipslib;

/**
 * Utility to convert between inches and metric units.
 */
public class Inches {
    public static double toCM(double inches) {
        return inches * 2.54;
    }

    public static double toMM(double inches) {
        return inches * 25.4;
    }

    public static double toM(double inches) {
        return inches * 0.0254;
    }

    public static double fromCM(double cm) {
        return cm / 2.54;
    }

    public static double fromMM(double mm) {
        return mm / 25.4;
    }

    public static double fromM(double m) {
        return m / 0.0254;
    }
}
