package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Performs spline interpolation given a set of control points.
 * <p>
 * Utility from FTCLib
 * <a href="https://github.com/FTCLib/FTCLib/blob/master/core/src/main/java/com/arcrobotics/ftclib/util/InterpLUT.java">Source</a>
 */
public class InterpolatedLookupTable {

    private List<Double> mX = new ArrayList<>();
    private List<Double> mY = new ArrayList<>();
    private List<Double> mM = new ArrayList<>();

    /**
     * Creates a new instance of InterpolatedLookupTable to add values to.
     */
    public InterpolatedLookupTable() {
    }

    /**
     * Adds a control point to the lookup table.
     *
     * @param input  x
     * @param output y
     */
    public void add(double input, double output) {
        mX.add(input);
        mY.add(output);
    }

    /**
     * Creates a monotone cubic spline from a given set of control points.
     *
     * <p>
     * The spline is guaranteed to pass through each control point exactly. Moreover, assuming the control points are
     * monotonic (Y is non-decreasing or non-increasing) then the interpolated values will also be monotonic.
     *
     * @throws IllegalArgumentException if the X or Y arrays are null, have different lengths or have fewer than 2 values.
     */
    //public static LUTWithInterpolator createLUT(List<Double> x, List<Double> y) {
    public void createLUT() {
        List<Double> x = mX;
        List<Double> y = mY;

        if (x == null || y == null || x.size() != y.size() || x.size() < 2) {
            throw new IllegalArgumentException("There must be at least two control "
                    + "points and the arrays must be of equal length.");
        }

        int n = x.size();
        Double[] d = new Double[n - 1]; // could optimize this out
        Double[] m = new Double[n];

        // Compute slopes of secant lines between successive points.
        for (int i = 0; i < n - 1; i++) {
            double h = x.get(i + 1) - x.get(i);
            if (h <= 0.0f) {
                throw new IllegalArgumentException("The control points must all "
                        + "have strictly increasing X values.");
            }
            d[i] = (y.get(i + 1) - y.get(i)) / h;
        }

        // Initialize the tangents as the average of the secants.
        m[0] = d[0];
        for (int i = 1; i < n - 1; i++) {
            m[i] = (d[i - 1] + d[i]) * 0.5f;
        }
        m[n - 1] = d[n - 2];

        // Update the tangents to preserve monotonicity.
        for (int i = 0; i < n - 1; i++) {
            if (d[i] == 0.0f) { // successive Y values are equal
                m[i] = 0.0;
                m[i + 1] = 0.0;
            } else {
                double a = m[i] / d[i];
                double b = m[i + 1] / d[i];
                double h = Math.hypot(a, b);
                if (h > 9.0f) {
                    double t = 3.0f / h;
                    m[i] = t * a * d[i];
                    m[i + 1] = t * b * d[i];
                }
            }
        }
        mX = x;
        mY = y;
        mM = Arrays.asList(m);
    }

    /**
     * Interpolates the value of Y = f(X) for given X. Clamps X to the domain of the spline.
     *
     * @param input The X value.
     * @return The interpolated Y = f(X) value.
     */
    public double get(double input) {
        // Handle the boundary cases.
        int n = mX.size();
        if (Double.isNaN(input)) {
            return input;
        }
        if (input <= mX.get(0)) {
            throw new IllegalArgumentException("User requested value outside of bounds of LUT. Bounds are: " + mX.get(0).toString() + " to " + mX.get(n - 1).toString() + ". Value provided was: " + input);
        }
        if (input >= mX.get(n - 1)) {
            throw new IllegalArgumentException("User requested value outside of bounds of LUT. Bounds are: " + mX.get(0).toString() + " to " + mX.get(n - 1).toString() + ". Value provided was: " + input);
        }

        // Find the index 'i' of the last point with smaller X.
        // We know this will be within the spline due to the boundary tests.
        int i = 0;
        while (input >= mX.get(i + 1)) {
            i += 1;
            if (input == mX.get(i)) {
                return mY.get(i);
            }
        }

        // Perform cubic Hermite spline interpolation.
        double h = mX.get(i + 1) - mX.get(i);
        double t = (input - mX.get(i)) / h;
        return (mY.get(i) * (1 + 2 * t) + h * mM.get(i) * t) * (1 - t) * (1 - t)
                + (mY.get(i + 1) * (3 - 2 * t) + h * mM.get(i + 1) * (t - 1)) * t * t;
    }

    // For debugging.
    @NonNull
    @Override
    public String toString() {
        StringBuilder str = new StringBuilder();
        int n = mX.size();
        str.append("[");
        for (int i = 0; i < n; i++) {
            if (i != 0) {
                str.append(", ");
            }
            str.append("(").append(mX.get(i));
            str.append(", ").append(mY.get(i));
            str.append(": ").append(mM.get(i)).append(")");
        }
        str.append("]");
        return str.toString();
    }

}