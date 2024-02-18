package org.murraybridgebunyips.bunyipslib;

/**
 * Utility to convert between inches and other units.
 */
public class Inches {
    private Inches() {
    }

    public static double toCM(double inches) {
        return inches * 2.54;
    }

    public static double toMM(double inches) {
        return inches * 25.4;
    }

    public static double toM(double inches) {
        return inches * 0.0254;
    }

    public static double toFieldTiles(double inches) {
        return inches / 23;
    }

    public static double toHammerUnits(double inches) {
        return inches / 0.75;
    }

    public static double toFootballFields(double inches) {
        return inches * 3600.0;
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

    public static double fromFieldTiles(double tiles) {
        return tiles * 23;
    }

    public static double fromHammerUnits(double units) {
        return units * 0.75;
    }

    public static double fromFootballFields(double ff) {
        return ff / 3600.0;
    }
}
