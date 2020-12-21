package util.math.geometry;

import org.ejml.simple.SimpleMatrix;
import util.math.units.HALAngleUnit;

/**
 * The base interface for all vector classes.
 * <p>
 * Creation Date: 7/11/17
 *
 * @param <V> This vector datatype.
 * @author Cole Savage, Level Up
 * @version 3.0.0
 * @see BaseEuclideanVector
 * @see Vector2D
 * @see Vector3D
 * @since 1.0.0
 */
public interface Vector<V extends Vector<V>> {

    /**
     * Gets whether this vector is a zero vector.
     *
     * @return Whether this vector is a zero vector.
     */
    boolean isZeroVector();

    /**
     * Gets whether this vector is a unit vector.
     *
     * @return Whether this vector is a unit vector.
     */
    boolean isUnitVector();

    /**
     * Gets whether this vector is normal to the given vector.
     *
     * @param vector A second vector that may or may not be normal to this vector.
     * @return Whether this vector is normal to the given vector.
     */
    boolean isNormalTo(V vector);

    /**
     * Calculates the magnitude of the vector.
     *
     * @return The magnitude of the vector.
     */
    double magnitude();

    /**
     * Calculates the angle between this vector and the given vector.
     *
     * @param vector A second vector to calculate the angle to.
     * @param unit   The output units for the angle.
     * @return The angle between this vector and the given vector.
     */
    double angleTo(V vector, HALAngleUnit unit);

    /**
     * Normalizes this vector so that it becomes a unit vector.
     *
     * @return This vector.
     */
    V normalize();

    /**
     * Adds the given vector to this vector.
     *
     * @param vector The vector to add to this vector.
     * @return This vector.
     */
    V add(V vector);

    /**
     * Subtracts the given vector from this vector.
     *
     * @param vector The vector to subtract from this vector.
     * @return This vector.
     */
    V subtract(V vector);

    /**
     * Multiplies this vector by a scalar.
     *
     * @param scalar The scalar to multiply by.
     * @return This vector.
     */
    V multiply(double scalar);

    /**
     * Divides this vector by a scalar.
     *
     * @param scalar The scalar to divide by.
     * @return This vector.
     */
    V divide(double scalar);

    /**
     * Calculates the dot product of this vector with another vector.
     *
     * @param vector The vector to calculate the dot product with.
     * @return The dot product of the two vectors.
     */
    double dot(V vector);

    /**
     * Projects this vector onto the given vector.
     *
     * @param ontoVector The vector to project this vector onto.
     * @return The projection of this vector onto the given vector.
     */
    V project(V ontoVector);

    /**
     * Converts the vector into a matrix.
     *
     * @return The matrix representation of this vector.
     * @see SimpleMatrix
     */
    SimpleMatrix toMatrix();
}