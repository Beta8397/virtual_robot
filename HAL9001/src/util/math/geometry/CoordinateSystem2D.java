package util.math.geometry;

import java.util.function.Function;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

/**
 * An enum containing common 2D coordinate systems.
 * <p>
 * Creation Date: 5/27/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see CoordinateSystem
 * @see CoordinateSystem3D
 * @see Function
 * @since 1.1.0
 */
public enum CoordinateSystem2D implements CoordinateSystem<CoordinateSystem2D> {
    CARTESIAN, POLAR;

    @Override
    public int dimensionality() {
        return 2;
    }

    @Override
    public Function<double[], double[]> convertTo(CoordinateSystem2D coordinateSystem) {
        if (this.equals(coordinateSystem)) return (double[] point) -> point;
        else if (this.equals(CARTESIAN)) {
            return (double[] point) -> {
                double x = point[0];
                double y = point[1];
                return new double[]{hypot(x, y), atan2(y, x)};
            };
        }
        return (double[] point) -> {
            double r = point[0];
            double theta = point[1];
            return new double[]{r * cos(theta), r * sin(theta)};
        };
    }
}
