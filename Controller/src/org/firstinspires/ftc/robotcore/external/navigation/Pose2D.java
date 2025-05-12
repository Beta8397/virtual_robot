package org.firstinspires.ftc.robotcore.external.navigation;

import androidx.annotation.NonNull;

/**
 * Pose2D represents the position and heading of an object in 2D space.
 */
public class Pose2D {
    protected final double x;
    protected final double y;
    protected final DistanceUnit distanceUnit;
    protected final double heading;
    protected final AngleUnit headingUnit;

    /**
     * Creates a new Pose2D object.
     * @param distanceUnit the unit of distance for both x and y
     * @param x the x position
     * @param y the y position
     * @param headingUnit the unit of heading
     * @param heading the heading
     */
    public Pose2D(DistanceUnit distanceUnit,double x, double y, AngleUnit headingUnit, double heading) {
        this.x = x;
        this.y = y;
        this.distanceUnit = distanceUnit;
        this.heading = heading;
        this.headingUnit = headingUnit;
    }

    /**
     * This gets X in the desired distance unit
     * @param unit the desired distance unit
     * @return the X member converted to the desired distance unit
     */
    public double getX(DistanceUnit unit) {
        return unit.fromUnit(this.distanceUnit, x);
    }

    /**
     * This gets the Y in the desired distance unit
     * @param unit the desired distance unit
     * @return the Y member converted to the desired distance unit
     */
    public double getY(DistanceUnit unit) {
        return unit.fromUnit(this.distanceUnit, y);
    }

    /**
     * This gets the heading in the desired distance unit
     * Be aware that this normalizes the angle to be between -PI and PI for RADIANS or
     * between -180 and 180 for DEGREES
     * @param unit the desired distance unit
     * @return the heading converted to the desired Angle Unit
     */
    public double getHeading(AngleUnit unit) {
        return unit.fromUnit(this.headingUnit, heading);
    }

    /**
     * This returns a string representation of the object in a human readable format for debugging purposes.
     * @return a string representation of the object
     */
    @NonNull
    @Override
    public String toString() {
        return "(Pose2D) x=" + x + ", y=" + y + " " + distanceUnit + ", heading=" + heading + " " + headingUnit;
    }
}
