package util.math.geometry;

import org.ejml.simple.SimpleMatrix;
import util.exceptions.ExceptionChecker;
import util.exceptions.HALMathException;
import util.math.FakeNumpy;
import util.math.units.HALAngleUnit;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * A 2D Vector class.
 * <p>
 * Creation Date: 5/27/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Vector
 * @see BaseEuclideanVector
 * @see Vector2D
 * @see Point3D
 * @since 1.1.0
 */
public class Vector3D extends BaseEuclideanVector<Vector3D> {
    //The zero vector for 3 dimensions.
    private static final Vector3D ZERO_VECTOR = new Vector3D(0, 0, 0);

    /**
     * A constructor for Vector3D.
     *
     * @param start The vector's start point.
     * @param end The vector's end point.
     *
     * @see Point3D
     * @see FakeNumpy
     */
    public Vector3D(Point3D start, Point3D end) {
        super(FakeNumpy.subtract(end.coordinates, start.coordinates));
    }

    /**
     * A constructor for Vector3D. Assumes start point of the vector is the origin.
     *
     * @param end The vector's end point.
     *
     * @see Point3D
     */
    public Vector3D(Point3D end) {
        this(Point3D.getOrigin(), end);
    }

    /**
     * A constructor for Vector3D. Requires Cartesian components.
     *
     * @param x The x component.
     * @param y The y component.
     * @param z The z component.
     *
     * @see Point3D
     */
    public Vector3D(double x, double y, double z) {
        this(new Point3D(x, y, z));
    }

    /**
     * A constructor for Vector3D. Requires cylindrical components.
     *
     * @param r The r component.
     * @param theta The theta component.
     * @param angleUnit The units of the theta component.
     * @param z The z component.
     */
    public Vector3D(double r, double theta, HALAngleUnit angleUnit, double z) {
        this(new Point3D(r, theta, angleUnit, z));
    }

    /**
     * A constructor for Vector3D. Requires spherical components.
     *
     * @param rho The rho component.
     * @param phi The phi component.
     * @param phiUnit The units of the phi component.
     * @param theta The theta component.
     * @param thetaUnit The units of the theta component.
     */
    public Vector3D(double rho, double phi, HALAngleUnit phiUnit, double theta, HALAngleUnit thetaUnit) {
        this(new Point3D(rho, phi, phiUnit, theta, thetaUnit));
    }

    /**
     * A private Vector3D constructor used for cloning.
     *
     * @param v The vector to clone.
     */
    private Vector3D(Vector3D v) {
        components = v.components;
    }

    /**
     * Converts a 3D vector in (cartesian) matrix form into a Vector3D object.
     *
     * @param inputMatrix The (cartesian) matrix form of a 3D vector. Can be a column matrix or a row matrix.
     * @return The Vector3D representation of the given vector.
     *
     * @throws HALMathException Throws this exception if the input is not a vector matrix or if the input is not 3 dimensional.
     */
    public static Vector3D fromMatrix(SimpleMatrix inputMatrix) {
        ExceptionChecker.assertTrue(inputMatrix.isVector(), new HALMathException("Input matrix is not a vector matrix."));

        SimpleMatrix vectorMatrix;
        if (inputMatrix.numRows() == 1) vectorMatrix = inputMatrix.transpose();
        else vectorMatrix = inputMatrix.copy();

        ExceptionChecker.assertTrue(vectorMatrix.numRows() == CoordinateSystem3D.CARTESIAN.dimensionality(), new HALMathException("Input must be a 2D vector in matrix form."));

        return new Vector3D(vectorMatrix.get(0, 0), vectorMatrix.get(1, 0), vectorMatrix.get(2, 0));
    }

    /**
     * Gets the zero vector for 3 dimensions.
     *
     * @return The zero vector for 3 dimensions.
     */
    public static Vector3D getZeroVector() {
        return ZERO_VECTOR.clone();
    }

    /**
     * Gets the vector's x component.
     *
     * @return The vector's x component.
     */
    public double getX() {
        return components[0];
    }

    /**
     * Gets the vector's y component.
     *
     * @return The vector's y component.
     */
    public double getY() {
        return components[1];
    }

    /**
     * Gets the vector's y component.
     *
     * @return The vector's y component.
     */
    public double getZ() {
        return components[2];
    }

    /**
     * Gets the vector's r component.
     *
     * @return The vector's r component.
     */
    public double getR() {
        return new Point3D(components[0], components[1], components[2]).getR();
    }

    /**
     * Gets the vector's phi component (units are radians by default).
     *
     * @return The vector's phi component (units are radians by default).
     */
    public double getPhi() {
        return getPhi(HALAngleUnit.RADIANS);
    }

    /**
     * Gets the vector's phi component (units are radians by default).
     *
     * @return The vector's phi component (units are radians by default).
     */
    public double getPhi(HALAngleUnit angleUnit) {
        return new Point3D(components[0], components[1], components[2]).getPhi(angleUnit);
    }

    /**
     * Gets the vector's theta component (units are radians by default).
     *
     * @return The vector's theta component (units are radians by default).
     */
    public double getTheta() {
        return getTheta(HALAngleUnit.RADIANS);
    }

    /**
     * Gets the vector's theta component (units are radians by default).
     *
     * @return The vector's theta component (units are radians by default).
     */
    public double getTheta(HALAngleUnit angleUnit) {
        return new Point3D(components[0], components[1], components[2]).getTheta(angleUnit);
    }

    /**
     * Rotates this vector counterclockwise around the given 3D axis by the given angle. (counterclockwise is positive, clockwise is negative)
     *
     * @param angle        The angle to rotate this vector by.
     * @param angleUnit    The units of the given angle (units are radians by default).
     * @param rotationAxis The axis to rotate around.
     * @return This vector.
     */
    public Vector3D rotate(double angle, HALAngleUnit angleUnit, Axis3D rotationAxis) {
        double theta = angleUnit.convertTo(HALAngleUnit.RADIANS).apply(angle);
        double cosine = cos(theta);
        double sine = sin(theta);
        double oneMinusCosine = 1 - cosine;
        double x = rotationAxis.getX(), y = rotationAxis.getY(), z = rotationAxis.getZ();
        SimpleMatrix rotationMatrix = new SimpleMatrix(new double[][]{
                {cosine - x * x * oneMinusCosine, x * y * oneMinusCosine - z * sine, y * sine + x * z * oneMinusCosine},
                {z * sine + x * y * oneMinusCosine, cosine + y * y * oneMinusCosine, -x * sine + y * z * oneMinusCosine},
                {-y * sine + x * z * oneMinusCosine, x * sine + y * z * oneMinusCosine, cosine + z * z * oneMinusCosine}
        });
        this.setFromMatrix(rotationMatrix.mult(this.toMatrix()));
        return this;
    }

    /**
     * Rotates this vector counterclockwise around the given 3D axis by the given angle. (counterclockwise is positive, clockwise is negative)
     *
     * @param angle        The angle to rotate this vector by.
     * @param rotationAxis The axis to rotate around.
     * @return This vector.
     */
    public Vector3D rotate(double angle, Axis3D rotationAxis) {
        return rotate(angle, HALAngleUnit.RADIANS, rotationAxis);
    }

    /**
     * Calculates the cross product of this vector with the given vector.
     *
     * @param vector The vector to multiply this vector by (cross product).
     * @return The cross product of this vector with the given vector.
     */
    public Vector3D cross(Vector3D vector) {
        return new Vector3D(
                this.getY() * vector.getZ() - this.getZ() * vector.getY(),
                this.getZ() * vector.getX() - this.getX() * vector.getZ(),
                this.getX() * vector.getY() - this.getY() * vector.getX()
        );
    }

    @Override
    public Vector3D clone() {
        return new Vector3D(this);
    }
}