package com.acmerobotics.dashboard.canvas;

public abstract class CanvasOp {
    public enum Type {

        CIRCLE,


        POLYGON,


        POLYLINE,


        SPLINE,


        STROKE,


        FILL,


        STROKE_WIDTH;
    }

    private Type type;

    public CanvasOp(Type type) {
        this.type = type;
    }
}