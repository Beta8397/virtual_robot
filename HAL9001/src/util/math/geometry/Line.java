package util.math.geometry;

/**
 * A mathematical representation of a line. This differs from a vector because it has a defined start and end point.
 * <p>
 * Creation Date: 5/27/20
 *
 * @param <V> The type of vector associated with this line (should match dimensionality of the point).
 * @param <P> The type of point associated with this line (should match dimensionality of the vector).
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Vector
 * @see Point
 * @see BaseEuclideanVector
 * @see BaseEuclideanPoint
 * @since 1.1.0
 */
public class Line<V extends BaseEuclideanVector<V>, P extends BaseEuclideanPoint<V, P>> {
    //The line's start and end points.
    protected P startPoint, endPoint;
    //The vector indicating the direction of the line.
    protected V vector;

    /**
     * The constructor for a line.
     *
     * @param startPoint The starting point of the line.
     * @param endPoint   The ending point of the line.
     */
    public Line(P startPoint, P endPoint) {
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        vector = startPoint.vectorTo(endPoint);
    }

    /**
     * Calculates the shortest distance from a line to a point.
     *
     * @param point The point to calculate the distance to.
     * @return The shortest distance from this line to that point.
     */
    public double distanceTo(P point) {
        return vector.subtract(startPoint.vectorTo(point).project(vector)).magnitude();
    }

    /**
     * Gets the length of the line.
     *
     * @return The length of the line.
     */
    public double length() {
        return startPoint.distanceTo(endPoint);
    }
}
