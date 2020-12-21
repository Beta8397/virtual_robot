package util.math.geometry;

/**
 * The base interface for all point classes.
 * <p>
 * Creation Date: 5/27/20
 *
 * @param <V> The type of vector associated with this type of point.
 * @param <P> This point datatype.
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see BaseEuclideanPoint
 * @see Point2D
 * @see Vector
 * @since 1.1.0
 */
public interface Point<V extends Vector<V>, P extends Point<V, P>> {

    /**
     * Gets the distance from this point to another point.
     *
     * @param point A point to calculate the distance to.
     * @return The distance from this point to another point.
     */
    double distanceTo(P point);

    /**
     * Creates a vector from this point to another point.
     *
     * @param point The vector's end point relative to this point.
     * @return A vector from this point to the given point.
     * @see Vector
     */
    V vectorTo(P point);
}
