package Autonomous;

/**
 * Created by robotics on 12/27/17.
 */

/*
    A class to create and use vectors for location based classes and functions
 */
public class HeadingVector {

    private double x = 0;
    private double y = 0;

    public HeadingVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public HeadingVector(){
    }

    public void calculateVector(double headingInDegrees, double magnitude) {
//        magnitude = Math.abs(magnitude);
        double degree = headingInDegrees;
        degree += 90;
        degree = 180 - degree;
        //Log.d("Degree", Double.toString(degree));
        x = magnitude* Math.cos(Math.toRadians(degree));
        y = magnitude* Math.sin(Math.toRadians(degree));
    }

    public double x() { return x; }
    public double y() { return y; }

    public double getHeading() {
        double degree = Math.toDegrees(Math.atan2(y, x)) - 90;
        degree = 360 - degree;
        if (degree > 360) degree -= 360;
        if (degree < 0) degree += 360;
        return degree;
     }

    public double getMagnitude(){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public HeadingVector addVector(HeadingVector vectorToAdd){
        return new HeadingVector(x + vectorToAdd.x(), y + vectorToAdd.y());
    }
    public HeadingVector addVectors(HeadingVector[] vectors){
        HeadingVector toReturn = vectors[0];
        for(int i = 1; i < vectors.length; i ++){
            toReturn = toReturn.addVector(vectors[i]);
        }
        return toReturn;
    }

    public HeadingVector subtractVector(HeadingVector vectorToSub){
        return new HeadingVector(x - vectorToSub.x(), y - vectorToSub.y());
    }
}
