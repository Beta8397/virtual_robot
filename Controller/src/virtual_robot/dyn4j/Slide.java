package virtual_robot.dyn4j;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.joint.PrismaticJoint;
import org.dyn4j.geometry.Vector2;
import virtual_robot.controller.VirtualField;

/**
 * Specialized PrismaticJoint that can be set at a specified position (in specified units),
 * with some position tolerance.
 */
public class Slide extends PrismaticJoint {

    private double maxForce = 100;  //Newtons
    private double speed = 10;  //Meters per sec
    private double tolerance = 0.005;  //Meters
    private double positionMeters = 0;
    private VirtualField.Unit defaultUnit;

    /**
     * Constructor -- uses default values of maxForce, speed, and tolerance
     * @param b1        VRBody 1
     * @param b2        VRBody 2
     * @param anchor    Anchor point (in world units, at the time of Slide creation
     * @param axis      Slide axis vector
     * @param unit      Distance unit for anchor point, and for setting defaultUnit
     */
    public Slide(Body b1, Body b2, Vector2 anchor, Vector2 axis, VirtualField.Unit unit) {
        super(b1, b2,
                anchor.product(VirtualField.conversionFactor(unit, VirtualField.Unit.METER)),
                axis.product(VirtualField.conversionFactor(unit, VirtualField.Unit.METER)));
        defaultUnit = unit;
        initialize();
    }

    /**
     * Constructor -- with specified values of maxForce, speed, and tolerance
     */
    public Slide(Body b1, Body b2, Vector2 anchor, Vector2 axis, VirtualField.Unit unit,
                 double maxForce, double speed, double tolerance){
        super(b1, b2,
            anchor.product(VirtualField.conversionFactor(unit, VirtualField.Unit.METER)),
            axis.product(VirtualField.conversionFactor(unit, VirtualField.Unit.METER)));
        defaultUnit = unit;
        this.maxForce = maxForce;
        this.speed = speed;
        this.tolerance = tolerance;
        initialize();
    }

    private void initialize(){
        this.setMotorEnabled(true);
        this.setMaximumMotorForce(maxForce);
        this.setLimitsEnabled(-tolerance, tolerance);
        this.setMotorSpeed(0);
    }

    /**
     * Set slide position, using the default units, relative to the position at the time of
     * Slide creation.
     */
    public void setPosition(double position){
        double newPositionMeters =
                position * VirtualField.conversionFactor(defaultUnit, VirtualField.Unit.METER);
        setMotorSpeed(speed * Math.signum(newPositionMeters - this.positionMeters));
        this.positionMeters = newPositionMeters;
        setLimits(this.positionMeters - tolerance, this.positionMeters + tolerance);
    }

    /**
     * Set slide position, using the specified units, relative to the position at the time of
     * Slide creation.
     * @param position
     * @param unit
     */
    public void setPosition(double position, VirtualField.Unit unit){
        setPosition(position * VirtualField.conversionFactor(unit, defaultUnit));
    }

    /**
     * Get current slide position in default units, relative to the position at the time of
     * Slide creation.
     * @return
     */
    public double getPosition(){
        return positionMeters
                * VirtualField.conversionFactor(VirtualField.Unit.METER, defaultUnit);
    }

    /**
     * Get current slide position in specified units, relative to the position at the time of
     * Slide creation.
     * @param unit
     * @return
     */
    public double getPosition(VirtualField.Unit unit){
        return positionMeters
                * VirtualField.conversionFactor(VirtualField.Unit.METER, unit);
    }

}
