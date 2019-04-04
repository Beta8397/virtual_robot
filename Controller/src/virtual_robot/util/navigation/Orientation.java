package virtual_robot.util.navigation;

/**
 * All the data, but none of the function, of the Orientation class in the FTC SDK
 */

public class Orientation {

    public final AxesReference axesReference;
    public final AxesOrder axesOrder;
    public final AngleUnit angleUnit;
    public final float firstAngle;
    public final float secondAngle;
    public final float thirdAngle;
    public final long acquisitionTime;

    public Orientation(AxesReference axesReference, AxesOrder axesOrder, AngleUnit angleUnit, float firstAngle,
                       float secondAngle, float thirdAngle, long acquisitionTime){
        this.axesReference = axesReference;
        this.axesOrder = axesOrder;
        this.angleUnit = angleUnit;
        this.firstAngle = firstAngle;
        this.secondAngle = secondAngle;
        this.thirdAngle = thirdAngle;
        this.acquisitionTime = System.nanoTime();
    }

}
