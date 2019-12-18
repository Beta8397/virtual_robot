package org.firstinspires.ftc.teamcode.ftc16072.Util;

public class Polar {

    private double theta;
    private double r;

    Polar(double theta, double r) {
        this.theta = theta;
        this.r = r;
    }

    public double getTheta() {
        return theta;
    }

    public double getDegrees() {
        return theta * 360 / (2 * Math.PI);
    }

    public double getR() {
        return r;
    }

    public static Polar fromCartesian(double x, double y) {
        double r = Math.hypot(x, y);
        double theta = Math.atan2(y, x);
        return new Polar(theta, r);
    }

    public void subtractAngle(double heading) {
        theta = theta - heading;
    }

    public double getX() {
        return r * Math.cos(theta);
    }

    public double getY() {
        return r * Math.sin(theta);
    }


}

