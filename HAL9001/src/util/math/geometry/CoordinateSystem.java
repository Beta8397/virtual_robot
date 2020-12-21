package util.math.geometry;

import java.util.function.Function;

/**
 * The base interface for all coordinate systems.
 * <p>
 * Creation Date: 5/27/20
 *
 * @param <T> This coordinate system datatype.
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see CoordinateSystem2D
 * @see CoordinateSystem3D
 * @see Function
 * @since 1.1.0
 */
public interface CoordinateSystem<T extends CoordinateSystem<T>> {

    /**
     * Gets the dimensionality of a coordinate system.
     *
     * @return The dimensionality of a coordinate system.
     */
    int dimensionality();

    /**
     * Converts between one coordinate system enum and another.
     *
     * @param coordinateSystem The coordinate system to convert to.
     * @return The function converting a set of coordinates in one system to a set of coordinates in another system.
     */
    Function<double[], double[]> convertTo(T coordinateSystem);
}