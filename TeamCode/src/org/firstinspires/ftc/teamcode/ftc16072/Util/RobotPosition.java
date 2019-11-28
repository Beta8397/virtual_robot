package org.firstinspires.ftc.teamcode.ftc16072.Util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotPosition {
    private double x_cm;
    private double y_cm;
    private double heading_radians;

    public RobotPosition(double x, double y, DistanceUnit du, double heading, AngleUnit au) {
        x_cm = du.toCm(x);
        y_cm = du.toCm(y);
        heading_radians = au.toRadians(heading);
    }

    public double getX(DistanceUnit du) {
        return du.fromCm(x_cm);
    }

    public double getY(DistanceUnit du) {
        return du.fromCm(y_cm);
    }

    public double getHeading(AngleUnit au) {
        return au.fromRadians(heading_radians);
    }
}