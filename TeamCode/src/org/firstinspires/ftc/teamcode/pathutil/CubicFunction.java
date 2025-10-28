package org.firstinspires.ftc.teamcode.pathutil;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Two dimensional parametric cubic function.
 * Function is of the form: (x,y) = ( ax + bx*s + cx*s*s + dx*s*s*s, ay + by*s + cy*s*s + dy*s*s*s )
 */
public class CubicFunction implements ParametricFunction {

    private float a, b, c, d;

    public CubicFunction(float a, float b, float c, float d){
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    /**
     * Return the position (x,y)
     * @param s
     * @return
     */
    @Override
    public float p(float s) { return ((d*s + c) * s + b) * s + a; }

    /**
     * Return the first derivative of position (x', y')
     * @param s
     * @return
     */
    @Override
    public float d1(float s) { return (d * 1.5f*s + c) * 2.0f * s + b; }

    /**
     * Return the second derivative of position (x", y")
     * @param s
     * @return
     */
    @Override
    public float d2(float s) { return (d * 3.0f * s + c) * 2.0f; }



}
