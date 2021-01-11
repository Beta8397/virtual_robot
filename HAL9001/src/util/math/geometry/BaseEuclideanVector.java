package util.math.geometry;

import org.ejml.simple.SimpleMatrix;
import util.exceptions.ExceptionChecker;
import util.exceptions.HALMathException;
import util.math.HALMathUtil;
import util.math.units.HALAngleUnit;

import java.util.Arrays;

import static java.lang.Math.acos;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

/**
 * The base class for all Euclidean vectors.
 * <p>
 * Creation Date: 7/11/17
 *
 * @param <V> This class's datatype.
 * @author Cole Savage, Level Up
 * @version 3.0.0
 * @see Vector
 * @since 1.1.0
 */
public abstract class BaseEuclideanVector<V extends BaseEuclideanVector<V>> implements Vector<V> {
    //The vector's components (cartesian).
    protected double[] components;

    /**
     * The base constructor for Euclidean vectors.
     *
     * @param components The (cartesian) components of a vector
     */
    public BaseEuclideanVector(double... components) {
        this.components = components;
    }

    @Override
    public boolean isZeroVector() {
        boolean isZero = true;
        for (double component : components) isZero &= component == 0;
        return isZero;
    }

    @Override
    public boolean isNormalTo(V vector) {
        return this.dot(vector) == 0;
    }

    @Override
    public boolean isUnitVector() {
        return this.magnitude() == 1;
    }

    @SuppressWarnings("unchecked")
    @Override
    public double magnitude() {
        return sqrt(this.dot((V) this));
    }

    @Override
    public double angleTo(V vector, HALAngleUnit angleUnit) {
        return HALAngleUnit.RADIANS.convertTo(angleUnit).apply(acos(this.dot(vector) / (this.magnitude() * vector.magnitude())));
    }

    /**
     * Gets the angle between two vectors. Defaults to radians.
     *
     * @param vector The vector to get the angle to.
     * @return The angle between two vectors, defaults to radians.
     */
    public double angleTo(V vector) {
        return angleTo(vector, HALAngleUnit.RADIANS);
    }

    @SuppressWarnings("unchecked")
    @Override
    public V normalize() {
        double norm = this.magnitude();
        if(this.isZeroVector()) return null;
        for (int i = 0; i < components.length; i++) components[i] /= norm;
        return (V) this;
    }

    @SuppressWarnings("unchecked")
    @Override
    public V add(V vector) {
        double[] otherComponents = vector.components;
        for (int i = 0; i < components.length; i++) components[i] += otherComponents[i];
        return (V) this;
    }

    @Override
    public V subtract(V vector) {
        return add(vector.multiply(-1));
    }

    @SuppressWarnings("unchecked")
    @Override
    public V multiply(double scalar) {
        for (int i = 0; i < components.length; i++) components[i] *= scalar;
        return (V) this;
    }

    @Override
    public V divide(double scalar) {
        return multiply(1 / scalar);
    }

    public V negate() {
        return this.multiply(-1);
    }

    public V scaleTo(double scale) {
        return this.normalize().multiply(scale);
    }

    @Override
    public double dot(V vector) {
        double[] otherComponents = vector.components;
        double total = 0;
        for (int i = 0; i < components.length; i++) total += components[i] * otherComponents[i];
        return total;
    }

    @Override
    public V project(V ontoVector) {
        return ontoVector.multiply(this.dot(ontoVector) / ontoVector.dot(ontoVector));
    }

    @Override
    public SimpleMatrix toMatrix() {
        double[][] vectorMatrix = new double[components.length][1];
        for (int i = 0; i < components.length; i++) vectorMatrix[i] = new double[]{components[i]};
        return new SimpleMatrix(vectorMatrix);
    }

    /**
     * Sets this vector object to the values contained in the given matrix vector. This is only used internally.
     *
     * @param inputMatrix THe matrix vector to set this vector to.
     * @throws HALMathException Throws this exception if the input matrix is not a vector matrix or is of the wrong dimensionality.
     */
    protected final void setFromMatrix(SimpleMatrix inputMatrix) {
        ExceptionChecker.assertTrue(inputMatrix.isVector(), new HALMathException("Input matrix is not a vector matrix."));

        SimpleMatrix vectorMatrix;
        if (inputMatrix.numRows() == 1) vectorMatrix = inputMatrix.transpose();
        else vectorMatrix = inputMatrix.copy();

        ExceptionChecker.assertTrue(vectorMatrix.numRows() == components.length, new HALMathException("Input must be a " + components.length + "D vector in matrix form."));

        for (int i = 0; i < components.length; i++) {
            components[i] = HALMathUtil.floatingPointFix(vectorMatrix.get(i, 0));
        }
    }

    /**
     * Rotates an N dimensional vector on N-2 dimensional object defined by two given orthonormal axes vectors.
     * Not very efficient, but works for all dimensions. Extensions of the base class may have more specialized rotation functions.
     *
     * @param u         The first vector defining the N-2 dimensional object.
     * @param v         The second vector defining the N-2 dimensional object.
     * @param angle     The angle to rotate.
     * @param angleUnit The units of the angle (defaults to radians).
     * @return This vector.
     * @throws HALMathException Throws this exception of the given axes vectors are not orthonormal.
     */
    @SuppressWarnings("unchecked")
    public final V rotate(Axis<V> u, Axis<V> v, double angle, HALAngleUnit angleUnit) {
        ExceptionChecker.assertTrue(u.getAxisVector().isNormalTo(v.getAxisVector()), new HALMathException("Vectors defining the rotation object must be orthonormal."));

        SimpleMatrix uMatrix = u.getAxisVector().toMatrix();
        SimpleMatrix vMatrix = v.getAxisVector().toMatrix();

        SimpleMatrix a = vMatrix.mult(uMatrix.transpose()).minus(uMatrix.mult(vMatrix.transpose()));
        SimpleMatrix b = uMatrix.mult(uMatrix.transpose()).plus(vMatrix.mult(vMatrix.transpose()));

        double theta = angleUnit.convertTo(HALAngleUnit.RADIANS).apply(angle);

        SimpleMatrix rotationMatrix = SimpleMatrix.identity(components.length).plus(a.scale(sin(theta))).plus(b.scale(cos(theta) - 1));
        this.setFromMatrix(rotationMatrix.mult(this.toMatrix()));

        return (V) this;
    }


    /**
     * Rotates an N dimensional vector on N-2 dimensional object defined by two given orthonormal axes vectors.
     * Not very efficient, but works for all dimensions. Extensions of the base class may have more specialized rotation functions.
     *
     * @param u     The first vector defining the N-2 dimensional object.
     * @param v     The second vector defining the N-2 dimensional object.
     * @param angle The angle to rotate (in radians).
     * @return This vector.
     * @throws HALMathException Throws this exception of the given axes vectors are not orthonormal.
     */
    public V rotate(Axis<V> u, Axis<V> v, double angle) {
        return rotate(u, v, angle, HALAngleUnit.RADIANS);
    }

    @Override
    public String toString() {
        String componentArrayString = Arrays.toString(components);
        return "<" + componentArrayString.substring(1, componentArrayString.length() - 1) + '>';
    }

    /**
     * Clones the vector.
     *
     * @return A copy of this vector.
     */
    public abstract V clone();
}