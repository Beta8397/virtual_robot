package org.firstinspires.ftc.teamcode.pathutil;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Two dimensional parametric cubic function.
 * Function is of the form: (x,y) = ( ax + bx*s + cx*s*s + dx*s*s*s, ay + by*s + cy*s*s + dy*s*s*s )
 */
public class CubicFunction2D implements ParametricFunction2D {

    private VectorF a, b, c, d;

    public CubicFunction2D(VectorF a, VectorF b, VectorF c, VectorF d){
        this.a = new VectorF(a.get(0), a.get(1));
        this.b = new VectorF(b.get(0), b.get(1));
        this.c = new VectorF(c.get(0), c.get(1));
        this.d = new VectorF(d.get(0), d.get(1));
    }

    /**
     * Return the position (x,y)
     * @param s
     * @return
     */
    @Override
    public VectorF p(float s) {
        return d.multiplied(s).added(c).multiplied(s).added(b).multiplied(s).added(a);
    }

    /**
     * Return the first derivative of position (x', y')
     * @param s
     * @return
     */
    @Override
    public VectorF d1(float s) {
        return d.multiplied(1.5f*s).added(c).multiplied(2.0f*s).added(b);
    }

    /**
     * Return the second derivative of position (x", y")
     * @param s
     * @return
     */
    @Override
    public VectorF d2(float s) {
        return d.multiplied(3.0f*s).added(c).multiplied(2.0f);
    }

    /**
     * Return the square of the distance from point (x0, y0) to the point on the curve p(s)
     * @param x0
     * @param y0
     * @param s
     * @return
     */
    public float distSquared(float x0, float y0, float s){
        VectorF delta = p(s).subtracted(new VectorF(x0, y0));
        return delta.dotProduct(delta);
    }

    /**
     * Iteratively (Newton's method) calculate the parameter s corresponding to the point
     * @param x0
     * @param y0
     * @param s0
     * @return
     */
    protected double findClosestPt(double x0, double y0, double s0) {
        double epsilon = .0001f;
        double delta = 100;
        int maxIter = 50;
        int numIter = 0;
        while(delta > epsilon && numIter<maxIter) {
            numIter++;
            if (numIter%10 == 0) break;
            VectorF p = p((float)s0);
            VectorF d1 = d1((float)s0);
            VectorF d2 = d2((float)s0);
            double f = (p.get(0) - x0) * d1.get(0) + (p.get(1) - y0) * d1.get(1);
            double fDeriv = (p.get(0) - x0) * d2.get(0) + d1.get(0) * d1.get(0) + (p.get(1) - y0) * d2.get(1) + d1.get(1) * d1.get(1);
            double s = s0 - f / fDeriv;
            delta = Math.abs(s0 - s);
            s0 = s;
        }
        return numIter<maxIter? s0 : Double.MAX_VALUE;
    }


}
