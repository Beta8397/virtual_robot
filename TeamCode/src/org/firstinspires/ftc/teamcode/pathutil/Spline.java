package org.firstinspires.ftc.teamcode.pathutil;

import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Spline implements Curve {
    private CubicSpline2D spline;
    private double startDir;
    private double endDir;
    private PathConstraints pathConstraints;
    private List<Pose> controlPoints = new ArrayList<>();
    private List<FuturePose> futureControlPoints = new ArrayList<>();
    private boolean initialized = false;
    private boolean resetClosestTValueFlag = false;

    public Spline(List<Pose> controlPoints, double startDir, double endDir, PathConstraints pathConstraints){
        this.controlPoints = controlPoints;
        this.startDir = startDir;
        this.endDir = endDir;
        this.pathConstraints = pathConstraints;
        initialize();
    }

    public Spline(double startDir, double endDir, PathConstraints pathConstraints, Pose... poses){
        this(new ArrayList<>(Arrays.asList(poses)), startDir, endDir, pathConstraints);
    }

    public Spline(double startDir, double endDir, Pose... poses){
        this(new ArrayList<>(Arrays.asList(poses)), startDir, endDir, PathConstraints.defaultConstraints);
    }

    public Spline(double startDir, double endDir, PathConstraints pathConstraints, FuturePose... futurePoses){
        this.startDir = startDir;
        this.endDir = endDir;
        this.pathConstraints = pathConstraints;

        boolean lazyInitialize = false;
        ArrayList<Pose> initializedControlPoints = new ArrayList<>();
        for (FuturePose pose : futurePoses) {
            if (!pose.initialized()) {
                lazyInitialize = true;
                break;
            }

            initializedControlPoints.add(pose.getPose());
        }

        if (lazyInitialize) {
            this.controlPoints = new ArrayList<>();
            this.futureControlPoints = new ArrayList<>(Arrays.asList(futurePoses));
        } else {
            this.controlPoints = initializedControlPoints;
            initialize();
        }
    }


    @Override
    public void initialize() {
        resetClosestTValueFlag = true;
        if (initialized) return;

        if (controlPoints.isEmpty() && !futureControlPoints.isEmpty()) {
            for (FuturePose futurePose : futureControlPoints) {
                controlPoints.add(futurePose.getPose());
            }
            futureControlPoints.clear();
        }
        this.spline = new CubicSpline2D(controlPoints, (float)startDir, (float)endDir);
        initialized = true;
    }

    @Override
    public double length(){
        return spline.getTotalPathLength();
    }

    @Override
    public Vector getNormalVector(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Vector v = getDerivative(t);         // velocity
        Vector a = getSecondDerivative(t);   // acceleration

        double vMag = v.getMagnitude();
        if (vMag == 0) return new Vector(0, 0); // no tangent direction

        Vector tangent = v.normalize();

        // Normal component of acceleration
        Vector aNormal = a.minus(tangent.times(a.dot(tangent)));

        double aNormalMag = aNormal.getMagnitude();
        if (aNormalMag == 0) return new Vector(0, 0); // straight line, no principal normal

        return aNormal.normalize();
    }

    @Override
    public Curve getReversed() {
        return null;
    }

    @Override
    public Vector getDerivative(double t) {
        t = MathFunctions.clamp(t, 0, 1);
        VectorF d1 = spline.d1(t);
        return new Vector(new Pose(d1.get(0), d1.get(1)));
    }

    @Override
    public Pose getPose(double t) {
        t = MathFunctions.clamp(t, 0, 1);
        VectorF p = spline.p(t);
        return new Pose(p.get(0), p.get(1));
    }

    @Override
    public Vector getSecondDerivative(double t) {
        t = MathFunctions.clamp(t, 0, 1);
        VectorF d2 = spline.d2(t);
        return new Vector(new Pose(d2.get(0), d2.get(1)));
    }

    @Override
    public ArrayList<Pose> getControlPoints() {
        return new ArrayList<Pose>(controlPoints);
    }

    @Override
    public void setPathConstraints(PathConstraints constraints) {
        this.pathConstraints = constraints;
    }

    @Override
    public PathConstraints getPathConstraints() {
        return pathConstraints;
    }

    @Override
    public double getPathCompletion(double t) {
        return spline.getPathLength(t)/spline.getTotalPathLength();
    }

    @Override
    public double getT(double pathCompletion) {
        double t0 = MathFunctions.clamp(pathCompletion, 0, 1);
        double epsilon = 0.0001;
        double delta = 100;
        double maxIter = 20;
        int iter = 0;
        double totalPathLength = spline.getTotalPathLength();
        while (iter < maxIter && Math.abs(delta) > epsilon){
            double f = spline.getPathLength(t0) / totalPathLength;
            double fDeriv = spline.getPathLengthDerivative(t0) / totalPathLength;
            delta = - f / fDeriv;
            t0 = MathFunctions.clamp(t0 + delta, 0, 1);
        }
        return t0;
    }

    @Override
    public double getCurvature(double t){
        return spline.getCurvature(t);
    }

    @Override
    public double getClosestPoint(Pose pose, int searchLimit, double initialTValueGuess){
        if (resetClosestTValueFlag){
            initialTValueGuess = 0;
            resetClosestTValueFlag = false;
        }
        return Curve.super.getClosestPoint(pose, searchLimit, initialTValueGuess);
    }
}
