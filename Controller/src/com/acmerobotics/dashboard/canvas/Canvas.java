package com.acmerobotics.dashboard.canvas;

import java.util.ArrayList;
import java.util.List;

public class Canvas {

    private List<CanvasOp> ops = new ArrayList<CanvasOp>();

    public Canvas() {}

    public Canvas strokeCircle(double x, double y, double radius) {
        return this;
    }

    public Canvas fillCircle(double x, double y, double radius) {
        return this;
    }

    public Canvas strokePolygon(double[] xPoints, double[] yPoints) {
        return this;
    }

    public Canvas fillPolygon(double[] xPoints, double[] yPoints) {
        return this;
    }

    public Canvas strokePolyline(double[] xPoints, double[] yPoints) {
        return this;
    }

    public Canvas strokeLine(double x1, double y1, double x2, double y2) {
        return this;
    }

    public Canvas fillRect(double x, double y, double width, double height) {
        return this;
    }

    public Canvas strokeRect(double x, double y, double width, double height) {
        return this;
    }

    @Deprecated
    public Canvas strokeSpline(double ax, double bx, double cx, double dx, double ex, double fx,
                               double ay, double by, double cy, double dy, double ey, double fy) {
        return this;
    }

    public Canvas setFill(String color) {
        return this;
    }

    public Canvas setStroke(String color) {
        return this;
    }

    public Canvas setStrokeWidth(int width) {
        return this;
    }

    public List<CanvasOp> getOperations() {
        return ops;
    }

    public void clear() {
    }
}