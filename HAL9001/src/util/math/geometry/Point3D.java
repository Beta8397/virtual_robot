package util.math.geometry;

import util.math.units.HALAngleUnit;

/**
 * A 3D point class.
 * <p>
 * Creation Date: 10/1/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see BaseEuclideanPoint
 * @see Vector2D
 * @since 1.1.0
 */
public class Point3D extends BaseEuclideanPoint<Vector3D, Point3D> {

    /**
     * The most general constructor for Point3D. Works for cartesian, cylindrical, and spherical coordinates.
     *
     * @param a                The first coordinate.
     * @param b                The second coordinate.
     * @param c                The third coordinate.
     * @param coordinateSystem The 3D coordinate system being used to create the point.
     * @see CoordinateSystem
     * @see CoordinateSystem3D
     */
    public Point3D(double a, double b, double c, CoordinateSystem3D coordinateSystem) {
        super(coordinateSystem.convertTo(CoordinateSystem3D.CARTESIAN).apply(new double[]{a, b, c}));
    }

    /**
     * A simple Cartesian constructor for Point3D.
     *
     * @param x The x coordinate.
     * @param y The y coordinate.
     * @param z The z coordinate.
     * @see CoordinateSystem
     * @see CoordinateSystem3D
     */
    public Point3D(double x, double y, double z) {
        this(x, y, z, CoordinateSystem3D.CARTESIAN);
    }

    /**
     * A simple cylindrical constructor for Point3D.
     *
     * @param r         The r value.
     * @param theta     The theta value.
     * @param angleUnit The angle unit used to measure theta.
     * @param z         The z value.
     */
    public Point3D(double r, double theta, HALAngleUnit angleUnit, double z) {
        this(r, angleUnit.convertTo(HALAngleUnit.RADIANS).apply(theta), z, CoordinateSystem3D.CYLINDRICAL);
    }

    /**
     * A simple spherical constructor for Point3D.
     *
     * @param rho       The rho value.
     * @param phi       The phi value.
     * @param phiUnit   The angle unit used to measure phi.
     * @param theta     The theta value.
     * @param thetaUnit The angle unit used to measure theta.
     */
    public Point3D(double rho, double phi, HALAngleUnit phiUnit, double theta, HALAngleUnit thetaUnit) {
        this(rho, phiUnit.convertTo(HALAngleUnit.RADIANS).apply(phi), thetaUnit.convertTo(HALAngleUnit.RADIANS).apply(theta), CoordinateSystem3D.SPHERICAL);
    }

    /**
     * Private Point3D constructor used for cloning.
     *
     * @param point The point to clone.
     */
    private Point3D(Point3D point) {
        this.coordinates = point.coordinates;
    }

    /**
     * Gets a clone of the origin point.
     *
     * @return The origin point.
     */
    public static Point3D getOrigin() {
        return new Point3D(0, 0, 0);
    }

    /**
     * Gets the point's x value.
     *
     * @return The point's x value.
     */
    public double getX() {
        return coordinates[0];
    }

    /**
     * Sets the point's x value.
     *
     * @param x The new x value for the point.
     */
    public void setX(double x) {
        coordinates[0] = x;
    }

    /**
     * Gets the point's y value.
     *
     * @return The point's y value.
     */
    public double getY() {
        return coordinates[1];
    }

    /**
     * Sets the point's y value.
     *
     * @param y The new y value for the point.
     */
    public void setY(double y) {
        coordinates[1] = y;
    }

    /**
     * Gets the point's z value.
     *
     * @return The point's z value.
     */
    public double getZ() {
        return coordinates[2];
    }

    /**
     * Sets the point's z value.
     *
     * @param z The new z value for the point.
     */
    public void setZ(double z) {
        coordinates[2] = z;
    }

    /**
     * Gets the point's r value.
     *
     * @return The point's r value.
     */
    public double getR() {
        return CoordinateSystem3D.CARTESIAN.convertTo(CoordinateSystem3D.CYLINDRICAL).apply(coordinates)[0];
    }

    /**
     * Sets the point's r value.
     *
     * @param r The point's new r value.
     */
    public void setR(double r) {
        double[] cylindricalPoint = CoordinateSystem3D.CARTESIAN.convertTo(CoordinateSystem3D.CYLINDRICAL).apply(coordinates);
        cylindricalPoint[0] = r;
        coordinates = CoordinateSystem3D.CYLINDRICAL.convertTo(CoordinateSystem3D.CARTESIAN).apply(cylindricalPoint);
    }

    /**
     * Gets the point's theta value (default units are radians).
     *
     * @return The point's theta value (default units are radians).
     */
    public double getTheta() {
        return getTheta(HALAngleUnit.RADIANS);
    }

    /**
     * Sets the points theta value (default units are radians).
     *
     * @param theta The new theta value (default units are radians).
     */
    public void setTheta(double theta) {
        setTheta(theta, HALAngleUnit.RADIANS);
    }

    /**
     * Gets the point's theta value (default unit is radians).
     *
     * @param angleUnit The units of the returned angle.
     * @return The point's theta value (default unit is radians).
     */
    public double getTheta(HALAngleUnit angleUnit) {
        return HALAngleUnit.RADIANS.convertTo(angleUnit).apply(
                CoordinateSystem3D.CARTESIAN.convertTo(CoordinateSystem3D.CYLINDRICAL).apply(coordinates)[1]
        );
    }

    /**
     * Sets the points theta value (default units are radians).
     *
     * @param theta     The new theta value (default units are radians).
     * @param angleUnit The units of the given angle.
     */
    public void setTheta(double theta, HALAngleUnit angleUnit) {
        double[] cylindricalPoint = CoordinateSystem3D.CARTESIAN.convertTo(CoordinateSystem3D.CYLINDRICAL).apply(coordinates);
        cylindricalPoint[1] = angleUnit.convertTo(HALAngleUnit.RADIANS).apply(theta);
        coordinates = CoordinateSystem3D.CYLINDRICAL.convertTo(CoordinateSystem3D.CARTESIAN).apply(cylindricalPoint);
    }

    /**
     * Gets point's rho value.
     *
     * @return The point's rho value.
     */
    public double getRho() {
        return CoordinateSystem3D.CARTESIAN.convertTo(CoordinateSystem3D.SPHERICAL).apply(coordinates)[0];
    }

    /**
     * Gets the point's phi value (units default to radians).
     *
     * @return The point's phi value (default units are radians).
     */
    public double getPhi() {
        return getPhi(HALAngleUnit.RADIANS);
    }

    /**
     * Gets the point's phi value (units default to radians).
     *
     * @param angleUnit The unit of the angle that is returned.
     * @return The point's phi value (default units are radians).
     */
    public double getPhi(HALAngleUnit angleUnit) {
        return CoordinateSystem3D.CARTESIAN.convertTo(CoordinateSystem3D.SPHERICAL).apply(coordinates)[2];
    }

    @Override
    public Vector3D vectorTo(Point3D point) {
        return new Vector3D(this, point);
    }

    @Override
    public Point3D clone() {
        return new Point3D(this);
    }
}