package util.math.geometry;

import util.math.units.HALAngleUnit;

/**
 * A 2D point class.
 * <p>
 * Creation Date: 5/27/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see BaseEuclideanPoint
 * @see Vector2D
 * @since 1.1.0
 */
public class Point2D extends BaseEuclideanPoint<Vector2D, Point2D> {

    /**
     * The most general constructor for Point2D. Works for both polar and cartesian.
     *
     * @param a The first coordinate.
     * @param b The second coordinate.
     * @param coordinateSystem The 2D coordinate system being used to create the point.
     *
     * @see CoordinateSystem
     * @see CoordinateSystem2D
     */
    public Point2D(double a, double b, CoordinateSystem2D coordinateSystem) {
        super(coordinateSystem.convertTo(CoordinateSystem2D.CARTESIAN).apply(new double[]{a, b}));
    }

    /**
     * A simple Cartesian constructor for Point2D.
     *
     * @param x The x coordinate.
     * @param y The y coordinate.
     *
     * @see CoordinateSystem
     * @see CoordinateSystem2D
     */
    public Point2D(double x, double y) {
        this(x, y, CoordinateSystem2D.CARTESIAN);
    }

    /**
     * A simple Polar constructor for Point2D.
     *
     * @param r The r value.
     * @param theta The theta value.
     * @param angleUnit The angle unit used to measure theta.
     */
    public Point2D(double r, double theta, HALAngleUnit angleUnit) {
        this(r, angleUnit.convertTo(HALAngleUnit.RADIANS).apply(theta), CoordinateSystem2D.POLAR);
    }

    /**
     * Private Point2D constructor used for cloning.
     *
     * @param point The point to clone.
     */
    private Point2D(Point2D point) {
        this.coordinates = point.coordinates;
    }

    /**
     * Gets a clone of the origin point.
     *
     * @return The origin point.
     */
    public static Point2D getOrigin() {
        return new Point2D(0, 0);
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
     * Gets the point's r value.
     *
     * @return The point's r value.
     */
    public double getR() {
        return CoordinateSystem2D.CARTESIAN.convertTo(CoordinateSystem2D.POLAR).apply(coordinates)[0];
    }

    /**
     * Sets the point's r value.
     *
     * @param r The point's new r value.
     */
    public void setR(double r) {
        double[] polarPoint = CoordinateSystem2D.CARTESIAN.convertTo(CoordinateSystem2D.POLAR).apply(coordinates);
        polarPoint[0] = r;
        coordinates = CoordinateSystem2D.POLAR.convertTo(CoordinateSystem2D.CARTESIAN).apply(polarPoint);
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
                CoordinateSystem2D.CARTESIAN.convertTo(CoordinateSystem2D.POLAR).apply(coordinates)[1]
        );
    }

    /**
     * Sets the points theta value (default units are radians).
     *
     * @param theta The new theta value (default units are radians).
     * @param angleUnit The units of the given angle.
     */
    public void setTheta(double theta, HALAngleUnit angleUnit) {
        double[] polarPoint = CoordinateSystem2D.CARTESIAN.convertTo(CoordinateSystem2D.POLAR).apply(coordinates);
        polarPoint[0] = angleUnit.convertTo(HALAngleUnit.RADIANS).apply(theta);
        coordinates = CoordinateSystem2D.POLAR.convertTo(CoordinateSystem2D.CARTESIAN).apply(polarPoint);
    }

    @Override
    public Vector2D vectorTo(Point2D point) {
        return new Vector2D(this, point);
    }

    @Override
    public Point2D clone() {
        return new Point2D(this);
    }
}