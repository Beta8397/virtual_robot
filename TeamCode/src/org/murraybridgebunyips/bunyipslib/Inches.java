package org.murraybridgebunyips.bunyipslib;

/**
 * Utility to convert between inches and other units.
 */
public final class Inches {
    private Inches() {
    }

    /**
     * Converts inches to centimeters.
     *
     * @param inches the number of inches
     * @return the number of centimeters
     */
    public static double toCM(double inches) {
        return inches * 2.54;
    }

    /**
     * Converts inches to millimeters.
     *
     * @param inches the number of inches
     * @return the number of millimeters
     */
    public static double toMM(double inches) {
        return inches * 25.4;
    }

    /**
     * Converts inches to meters.
     *
     * @param inches the number of inches
     * @return the number of meters
     */
    public static double toM(double inches) {
        return inches * 0.0254;
    }

    /**
     * Converts inches to field tiles.
     *
     * @param inches the number of inches
     * @return the number of field tiles
     */
    public static double toFieldTiles(double inches) {
        return inches / 23;
    }

    /**
     * Converts inches to hammer units.
     *
     * @param inches the number of inches
     * @return the number of hammer units
     */
    public static double toHammerUnits(double inches) {
        return inches / 0.75;
    }

    /**
     * Converts inches to football fields.
     *
     * @param inches the number of inches
     * @return the number of football fields
     */
    public static double toFootballFields(double inches) {
        return inches * 3600.0;
    }

    /**
     * Converts inches to light years.
     *
     * @param inches the number of inches
     * @return the number of light years
     */
    public static double toLightYears(double inches) {
        return inches / 3.725e+17;
    }

    /**
     * Converts centimeters to inches.
     *
     * @param cm the number of centimeters
     * @return the number of inches
     */
    public static double fromCM(double cm) {
        return cm / 2.54;
    }

    /**
     * Converts millimeters to inches.
     *
     * @param mm the number of millimeters
     * @return the number of inches
     */
    public static double fromMM(double mm) {
        return mm / 25.4;
    }

    /**
     * Converts meters to inches.
     *
     * @param m the number of meters
     * @return the number of inches
     */
    public static double fromM(double m) {
        return m / 0.0254;
    }

    /**
     * Converts field tiles to inches.
     *
     * @param tiles the number of field tiles
     * @return the number of inches
     */
    public static double fromFieldTiles(double tiles) {
        return tiles * 23;
    }

    /**
     * Converts hammer units to inches.
     *
     * @param units the number of hammer units
     * @return the number of inches
     */
    public static double fromHammerUnits(double units) {
        return units * 0.75;
    }

    /**
     * Converts football fields to inches.
     *
     * @param ff the number of football fields
     * @return the number of inches
     */
    public static double fromFootballFields(double ff) {
        return ff / 3600.0;
    }

    /**
     * Converts light years to inches.
     *
     * @param ly the number of light years
     * @return the number of inches
     */
    public static double fromLightYears(double ly) {
        return ly * 3.725e+17;
    }
}
