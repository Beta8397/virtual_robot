package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import util.math.geometry.Point2D;
import util.math.geometry.Point3D;
import util.math.units.HALDistanceUnit;

public enum FieldLocations {
    BOTTOM_LEFT_CORNER(new Point3D(-72,-72,0)),
    BOTTOM_RIGHT_CORNER(new Point3D(72,-72,0)),
    TOP_LEFT_CORNER(new Point3D(-72,72,0)),
    TOP_RIGHT_CORNER(new Point3D(72,72,0)),
    LEFT_BLUE_WOBBLE_GOAL_START(new Point3D(-48,-48,0)),
    RIGHT_BLUE_WOBBLE_GOAL_START(new Point3D(-24,-48,0)),
    LEFT_RED_WOBBLE_GOAL_START(new Point3D(24,-48,0)),
    RIGHT_RED_WOBBLE_GOAL_START(new Point3D(48,-48,0)),
    BLUE_RING_START(new Point3D(-36,-24,0)),
    RED_RING_START(new Point3D(36,-24,0)),
    BLUE_LOWER_TARGET_ZONE(new Point3D(-60,12,0)),
    BLUE_MIDDLE_TARGET_ZONE(new Point3D(-36,36,0)),
    BLUE_UPPER_TARGET_ZONE(new Point3D(-60,60,0)),
    RED_LOWER_TARGET_ZONE(new Point3D(60,12,0)),
    RED_MIDDLE_TARGET_ZONE(new Point3D(36,36,0)),
    RED_UPPER_TARGET_ZONE(new Point3D(60,60,0)),
    LEFT_BLUE_POWERSHOT(new Point3D(-18.5,72,23.5)),
    MIDDLE_BLUE_POWERSHOT(new Point3D(-11,72,23.5)),
    RIGHT_BLUE_POWERSHOT(new Point3D(-3.5,72,23.5)),
    LEFT_RED_POWERSHOT(new Point3D(3.5,72,23.5)),
    MIDDLE_RED_POWERSHOT(new Point3D(11,72,23.5)),
    RIGHT_RED_POWERSHOT(new Point3D(18.5,72,23.5)),
    BLUE_LOWER_GOAL(new Point3D(-12,72,17)),
    BLUE_MIDDLE_GOAL(new Point3D(-12,72,27)),
    BLUE_HIGH_GOAL(new Point3D(-12,72,35.5)),
    RED_LOWER_GOAL(new Point3D(12,72,17)),
    RED_MIDDLE_GOAL(new Point3D(12,72,27)),
    RED_HIGH_GOAL(new Point3D(12,72,35.5));

    private final Point3D point;
    FieldLocations(Point3D point) {
        this.point = point;
    }

    @NotNull
    @Contract("_ -> new")
    public Point2D point2D(HALDistanceUnit distanceUnit) {
        return new Point2D(
                HALDistanceUnit.convert(point.getX(), HALDistanceUnit.INCHES, distanceUnit),
                HALDistanceUnit.convert(point.getY(), HALDistanceUnit.INCHES, distanceUnit)
        );
    }

    @NotNull
    @Contract(" -> new")
    public Point2D point2D() {
        return point2D(HALDistanceUnit.INCHES);
    }

    @NotNull
    @Contract("_ -> new")
    public Point3D point3D(HALDistanceUnit distanceUnit) {
        return new Point3D(
                HALDistanceUnit.convert(point.getX(), HALDistanceUnit.INCHES, distanceUnit),
                HALDistanceUnit.convert(point.getY(), HALDistanceUnit.INCHES, distanceUnit),
                HALDistanceUnit.convert(point.getZ(), HALDistanceUnit.INCHES, distanceUnit)
        );
    }

    @NotNull
    @Contract(" -> new")
    public Point3D point3D() {
        return point3D(HALDistanceUnit.INCHES);
    }

    @NotNull
    @Contract("_, _ -> new")
    public Pose2d toPose(double desiredHeading, HALDistanceUnit distanceUnit) {
        return new Pose2d(
                HALDistanceUnit.convert(point.getX(), HALDistanceUnit.INCHES, distanceUnit),
                HALDistanceUnit.convert(point.getY(), HALDistanceUnit.INCHES, distanceUnit),
                desiredHeading
        );
    }

    @NotNull
    @Contract("_ -> new")
    public Pose2d toPose(double desiredHeading) {
        return toPose(desiredHeading, HALDistanceUnit.INCHES);
    }
}
