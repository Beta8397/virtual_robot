package Autonomous;

/**
 * Created by root on 8/22/17.
 * A class for a location object
 */
public class Location {

    private double x;
    private double y;
    private double heading;

    public Location(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Location(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Location(Location target) {
        x = target.getX();
        y = target.getY();
        heading = target.getHeading();
    }

    public Location(Location location, double heading) {
        this(location.x, location.y, heading);
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    public Location setX(double x) { this.x = x; return this; }
    public Location setY(double y) { this.y = y; return this; }
    public Location setHeading(double heading) { this.heading = heading; return this; }

    public Location update(double x, double y) {
        this.x = x;
        this.y = y;
        return this;
    }

    public Location addX(double dx){ x += dx; return this; }
    public Location addY(double dy) { y += dy; return this; }

    public Location addXY(double dx, double dy) {
        x += dx;
        y += dy;
        return this;
    }

    public double distanceToLocation(Location location) {
        return Math.sqrt(Math.pow((location.getX() - getX()), 2) + Math.pow((location.getY() - getY()), 2));
    }

    public boolean withinRectangle(Rectangle area) {
        return (x >= area.left && x <= area.right && y >= area.top && y <= area.bottom);
    }

    public String toString() { return "X: " + x + ", Y: " + y + ", Heading:" + heading; }
}
