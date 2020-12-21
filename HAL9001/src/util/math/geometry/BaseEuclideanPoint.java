package util.math.geometry;

import static java.lang.Math.sqrt;

/**
 * The base class for all Euclidean points.
 * <p>
 * Creation Date: 5/27/20
 *
 * @param <V> The type of vector associated with this type of point.
 * @param <P> This point class datatype.
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Point
 * @see Vector
 * @see BaseEuclideanVector
 * @since 1.1.0
 */
public abstract class BaseEuclideanPoint<V extends BaseEuclideanVector<V>, P extends BaseEuclideanPoint<V, P>> implements Point<V, P> {
    //The coordinates of the point (cartesian).
    protected double[] coordinates;

    /**
     * The base constructor for Euclidean points.
     *
     * @param coordinates The coordinates of a point.
     */
    public BaseEuclideanPoint(double... coordinates) {
        this.coordinates = coordinates;
    }

    @Override
    public double distanceTo(P point) {
        double dst = 0;
        for (int i = 0; i < coordinates.length; i++) {
            dst += (coordinates[i] - point.coordinates[i]) * (coordinates[i] - point.coordinates[i]);
        }
        return sqrt(dst);
    }

    /**
     * Calculates the shortest distance from this point to a given line.
     *
     * @param line The line to calculate the distance to.
     * @return The shortest distance from this point to the given line.
     * @see Line
     */
    @SuppressWarnings("unchecked")
    public double distanceTo(Line<V, P> line) {
        return line.distanceTo((P) this);
    }

    /**
     * Clones the point.
     *
     * @return A copy of this point.
     */
    public abstract P clone();
}
