package virtual_robot.hardware.bno055;

import virtual_robot.controller.VirtualBot;
import virtual_robot.util.navigation.AxesOrder;
import virtual_robot.util.navigation.AxesReference;
import virtual_robot.util.navigation.Orientation;

public class BNO055IMUImpl implements BNO055IMU {
    private VirtualBot bot = null;
    private Parameters parameters = null;
    private double initialHeadingRadians = 0;
    private double headingRadians = 0;
    private boolean initialized = false;

    public BNO055IMUImpl(VirtualBot bot){
        this.bot = bot;
    }

    public synchronized boolean initialize(Parameters parameters){
        initialized = true;
        this.parameters = parameters;
        headingRadians = initialHeadingRadians = bot.getHeadingRadians();
        return true;
    }

    public synchronized Parameters getParameters() { return parameters; }

    public synchronized void close(){
        initialized = false;
        headingRadians = 0;
        initialHeadingRadians = 0;
    }

    public synchronized Orientation getAngularOrientation() {
        virtual_robot.util.navigation.AngleUnit angleUnit = parameters.angleUnit == AngleUnit.DEGREES ?
                virtual_robot.util.navigation.AngleUnit.DEGREES :
                virtual_robot.util.navigation.AngleUnit.RADIANS;
        return getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit);
    }

    public synchronized Orientation getAngularOrientation(AxesReference reference, AxesOrder order, virtual_robot.util.navigation.AngleUnit angleUnit) {
        if (!initialized) return null;

        double heading = headingRadians - initialHeadingRadians;
        if (heading > Math.PI) heading -= 2.0 * Math.PI;
        else if (heading < -Math.PI) heading += 2.0 * Math.PI;

        double piOver2;
        double firstAngle = 0.0, secondAngle = 0.0, thirdAngle = 0.0;
        if (angleUnit == virtual_robot.util.navigation.AngleUnit.DEGREES) {
            heading *= 180.0 / Math.PI;
            piOver2 = Math.PI / 2.0;
        } else {
            piOver2 = 90.0;
        }

        switch (order) {
            case ZXY: case ZXZ: case ZYX: case ZYZ:
                firstAngle = heading;
                break;
            case XZX: case XZY: case YZX: case YZY:
                secondAngle = heading;
                break;
            case XYZ: case YXZ:
                thirdAngle = heading;
                break;
            case YXY:
                secondAngle = heading;
                if (reference == AxesReference.INTRINSIC){
                    firstAngle = -piOver2;
                    thirdAngle = piOver2;
                } else {
                    firstAngle = piOver2;
                    thirdAngle = -piOver2;
                }
                break;
            case XYX:
                secondAngle = heading;
                if (reference == AxesReference.INTRINSIC){
                    firstAngle = piOver2;
                    thirdAngle = -piOver2;
                } else {
                    firstAngle = -piOver2;
                    thirdAngle = piOver2;
                }
        }
        return new Orientation(reference, order, angleUnit, (float)firstAngle, (float)secondAngle, (float)thirdAngle,
                System.nanoTime());

    }

    public synchronized void updateHeadingRadians( double heading ){
        headingRadians = heading;
    }
}
