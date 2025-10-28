package org.firstinspires.ftc.teamcode.pathutil;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import org.apache.commons.math3.linear.*;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.List;
import java.util.random.RandomGenerator;

/**
 * Represents a 2D Cubic Spline which has numSegments segments. Each segment is a parametric cubic function with
 * parameter s (0 <= s <= 1). There are numSegments+1 control points, and the spline passes through each of
 * those points. The points mark the beginning and end of each segment. The cubic segments (i.e., the coefficients
 * for each segment) are determined by applying the following rules:
 *
 * 1. Segment n evaluates to point n at s=0, and evaluates to point n+1 at s=1;
 * 2. The first and second deriviatives are continuous at the intermediate points;
 * 3. The client specifies the direction of travel at the beginning and end.
 */

public class CubicSpline2D {

    private List<Pose> poses;
    private int numSegments;
    private double startDir;
    private double endDir;
    private CubicFunction2D[] segments;
    private int index = 0;
    private double[] pathLengths = null;
    private CubicFunction[] pathLengthSegments = null;

    /**
     * Constructor
     * @param poses  The control points through which the spline must pass (x0, y0, x1, y1, x2, y2, etc.)
     * @param startDir The required initial direction of travel, in degrees.
     * @param endDir The required ending direction of travel, in degrees.
     */
    public CubicSpline2D(List<Pose> poses, double startDir, double endDir){
        this.startDir = startDir;
        this.endDir = endDir;
        numSegments = poses.size() - 1;
        VectorF[] points = new VectorF[numSegments+1];
        for (int i=0; i<=numSegments; i++){
            points[i] = new VectorF((float)poses.get(i).getX(), (float)poses.get(i).getY());
        }

        /**
         * The matrix equation to get the coefficients for all of the segments has the form:
         *
         *    MD = V ,
         *
         *    where M = Triadiagonal Matrix
         *    D = array of first derivatives at each control point
         *    V = input array, where each entry is a linear combination of control points.
         *
         *    Segments are numbered 0..(numSegments-1), and control points are numbered 0..numSegments
         *
         *    i.e., for 0<i<numSegments, v[i] = 3*(x[i+1] - x[i-1])
         *    For i=0, v[i] = derivative of segment 0 with respect to s, at s = 0
         *    For i=numSegments, v[i] = derivative of segment (numSegments-1) at s = 1.
         *
         *    Start by creating the tridiagonal matrix
         */

        double[][] mData = new double[numSegments+1][numSegments+1];

        for (int i=0; i<=numSegments; i++){
            if (i==0){
                mData[i][i] = 1.0;
            } else if (i == numSegments) {
                mData[i][i] = 1.0;
            } else {
                mData[i][i-1] = 1.0;
                mData[i][i] = 4.0;
                mData[i][i+1] = 1.0;
            }
        }

        RealMatrix matrix = new Array2DRowRealMatrix(mData);

        //Invert the tridiagonal matrix
        RealMatrix invMatrix = MatrixUtils.inverse(matrix);

        //Generate the input vectors (for x and y)
        double[] xData = new double[numSegments+1];
        double[] yData = new double[numSegments+1];

        //The first derivatives at the beginning and end ( D0 and DN are obtained from the begin and end travel directions)
        float startDist = points[1].subtracted(points[0]).magnitude();
        float endDist = points[numSegments].subtracted(points[numSegments-1]).magnitude();
        float D0x = (float)Math.cos(startDir) * startDist;
        float D0y = (float)Math.sin(startDir)*startDist;
        float DNx = (float)Math.cos(endDir) * endDist;
        float DNy = (float)Math.sin(endDir) * endDist;

        for (int i=0; i<=numSegments; i++){
            if (i==0){
                //Use this to specify the beginning derivative explicitly
                xData[i] = D0x;
                yData[i] = D0y;
            } else if (i==numSegments){
                //Use this to specify the ending derivative explicitly
                xData[i] = DNx;
                yData[i] = DNy;
            } else {
                xData[i] = 3.0 * (points[i+1].get(0) - points[i-1].get(0));
                yData[i] = 3.0 * (points[i+1].get(1) - points[i-1].get(1));
            }
        }

        RealVector xVec = new ArrayRealVector(xData);
        RealVector yVec = new ArrayRealVector(yData);

        //Compute the X and Y components of the first derivative for all control points
        RealVector Dx = invMatrix.operate(xVec);
        RealVector Dy = invMatrix.operate(yVec);

        /* From the control points themselves, and the first derivatives, compute the coefficients
         *   a, b, c, d
         * for each cubic function and create an array of cubic function objects
         */
        segments = new CubicFunction2D[numSegments];
        for (int i=0; i<numSegments; i++){
            VectorF a = points[i];
            VectorF b = new VectorF((float)Dx.getEntry(i), (float)Dy.getEntry(i));
            VectorF c = points[i+1].subtracted(points[i]).multiplied(3)
                    .subtracted(new VectorF((float)(2*Dx.getEntry(i)+Dx.getEntry(i+1)), (float)(2*Dy.getEntry(i)+Dy.getEntry(i+1))));
            VectorF d = points[i].subtracted(points[i+1]).multiplied(2)
                    .added(new VectorF((float)(Dx.getEntry(i)+Dx.getEntry(i+1)), (float)(Dy.getEntry(i)+Dy.getEntry(i+1))));
            segments[i] = new CubicFunction2D(a, b, c, d);
        }

        /*
         * Now that we have the array of 2D Cubic Functions, determine the cumulative path length at each control point
         */

        pathLengths = new double[numSegments+1];
        pathLengths[0] = 0;
        double pathLength = 0.0;
        VectorF p0 = segments[0].p(0);
        for (int i=1; i<= numSegments; i++){
            for (int j=1; j<=50; j++){
                float s = j/50.0f;
                VectorF p1 = segments[i-1].p(s);
                pathLength += p1.subtracted(p0).magnitude();
                p0 = p1;
            }
            pathLengths[i] = pathLength;
        }

        /*
         * Now create a 1D Cubic Spline so that cumulative path length can be estimated for any given segment and any
         * given value of parameter s.
         */

        //Initial and final derivatives of path length with respect to parameter s
        double D0_pathLength = Math.sqrt(D0x*D0x + D0y*D0y);
        double DN_pathLength = Math.sqrt(DNx*DNx + DNy*DNy);

        //Input vector for computing the 1D Cubic Spline for path length
        double[] pathData = new double[numSegments+1];
        for (int i=0; i<=numSegments; i++){
            if (i==0) {
                pathData[i] = D0_pathLength;
            } else if (i==numSegments) {
                pathData[i] = DN_pathLength;
            } else {
                pathData[i] = 3.0 * (pathLengths[i+1] - pathLengths[i-1]);
            }
        }
        RealVector pathLengthInputVector = new ArrayRealVector(pathData);

        //Compute the vector containing the first derivatives for path length
        RealVector D_pathLength = invMatrix.operate(pathLengthInputVector);

        //Compute the a, b, c, d coefficients for the cubic segments of the path length spline
        pathLengthSegments = new CubicFunction[numSegments];
        for (int i=0; i<numSegments; i++){
            float a = (float)pathLengths[i];
            float b = (float)D_pathLength.getEntry(i);
            float c = (float) (3.0 * (pathLengths[i + 1] - pathLengths[i])
                    - 2.0 * D_pathLength.getEntry(i) - D_pathLength.getEntry(i + 1));
            float d = (float)(-2.0 * (pathLengths[i + 1] - pathLengths[i])
                    + D_pathLength.getEntry(i) + D_pathLength.getEntry(i+1));
            pathLengthSegments[i] = new CubicFunction(a, b, c, d);
        }
    }


    /**
     * Return the CubicFunction2D object for the requested segment
     * @param i
     * @return
     */
    public CubicFunction2D getSegment(int i){
        return segments[i];
    }

    /**
     * Get the total number of segments
     * @return number of segments
     * Note:  number of control points = number of segments PLUS 1
     */
    public int getNumSegments() { return numSegments; }

    /**
     * Get position (x,y) corresponding to parameter s, for the requested segment.
     * @param i    Requested segment
     * @param s
     * @return     Position
     */
    public VectorF p(int i, float s) { return getSegment(i).p(s); }

    /**
     * Get the first derivative with respect to s for the requested segment
     * @param i    Requested segment
     * @param s
     * @return     First derivative
     */
    public VectorF d1(int i, float s){ return getSegment(i).d1(s); }

    /**
     * Get the second derivative with respect to s for the requested segment
     * @param i   Requested segment
     * @param s
     * @return    Second derivative
     */
    public VectorF d2(int i, float s) { return getSegment(i).d2(s); }

    /**
     * Given a previous value of s (s0), and a point (x0,y0), determine the new value of s that corresponds to
     * the closest point on the current segment to the point (x0,y0). If this value is greater than one (and current
     * segment is not the final segment), increment the segment index (i.e., move to next segment), and (using a seed
     * of 0) find the value of s that corresponds to the closest point on the next segment to (x0,y0). Return the
     * value of s. Note that the only circumstance under which a value s>1 will be returned is if the current segment
     * is the final segment, and the closest point is beyond the final control point.
     *
     * @param x0 x value of test point
     * @param y0 y value of test point
     * @param s0 previous s value (essentially a seed for the numeric computation.
     * @return s value giving point on spline that is closest to the test point
     */
    public double nextClosestPt(double x0, double y0, int k, double s0){
        double s = segments[k].findClosestPt(x0, y0, s0);
        if (s > 1 && index < numSegments-1){
            index++;
            s = segments[index].findClosestPt(x0, y0, 0);
        }
        return s;
    }

    public double nextClosestPt(double x0, double y0, double t0){
        int k = getK(t0);
        double s0 = getS(k, t0);
        double s = nextClosestPt(x0, y0, k, s0);
        if (s > 1 && k < numSegments-1) {
            k++;
            s0 = getS(k, t0);
            s = nextClosestPt(x0, y0, k, s0);
        }
        double t = Range.clip(getT(k,s), 0, 1);
        return t;
    }

    /**
     * Get estimated cumulative path length for parameter s at the specified spline segment
     * @param i    Segment index ( 0 <= i < numSegments )
     * @param s    Parameter (0 <= s <= 1)
     * @return     Cumulative path length
     */
    public float getPathLength(int i, float s){
        return pathLengthSegments[i].p(s);
    }

    /**
     * Get the estimated cumulative path length at the specified control point
     * @param i     Control point index (0 <= i <= numSegments)
     * @return
     */
    public float getPathLength(int i) { return (float)pathLengths[i]; }

    public float getPathLength(double t){
        t = MathFunctions.clamp(t, 0, 1);
        int k = getK(t);
        double s = getS(k, t);
        return getPathLength(k, (float)s);
    }

    public float getPathLengthDerivative(double t){
        t = MathFunctions.clamp(t, 0, 1);
        int k = getK(t);
        double s = getS(k, t);
        return pathLengthSegments[k].d1((float)s) * numSegments;
    }

    /**
     * Get the total path length for this spline
     * @return Total path length
     */
    public float getTotalPathLength() { return getPathLength(numSegments); }


    /**
     * Get heading change along the spline per unit path length.
     * Multiply this by travel speed to get heading change per unit time.
     * @param i         Spline segment
     * @param s         Parameter (0 <= s <= 1)
     * @return
     */
    public float headingChangePerUnitPathLength(int i, float s){
        VectorF d1 = d1(i, s);
        VectorF d2 = d2(i, s);
        return (d2.get(1) * d1.get(0) - d2.get(0) * d1.get(1)) / (float) Math.pow(d1.dotProduct(d1), 1.5f);
    }

    public float getCurvature(double t){
        VectorF d1 = d1(t);
        VectorF d2 = d2(t);
        return (d2.get(1) * d1.get(0) - d2.get(0) * d1.get(1)) / (float) Math.pow(d1.dotProduct(d1), 1.5f);
    }

    double getT(int k, double s){
        return ( k + s ) / numSegments;
    }

    int getK(double t){
        int k = (int)Math.floor(numSegments * t);
        return Range.clip(k, 0, numSegments-1);
    }

    double getS(int k, double t){
        return numSegments * t - k;
    }

    public VectorF p(double t){
        int k = getK(t);
        double s = getS(k, t);
        return p(k, (float)s);
    }

    public VectorF d1(double t){
        int k = getK(t);
        double s = getS(k, t);
        return d1(k, (float)s).multiplied(numSegments);
    }

    public VectorF d2(double t){
        int k = getK(t);
        double s = getS(k, t);
        return d2(k, (float)s).multiplied(numSegments*numSegments);
    }

}
