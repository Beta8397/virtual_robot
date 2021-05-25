package virtual_robot.dyn4j;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.joint.RevoluteJoint;
import org.dyn4j.geometry.Vector2;
import virtual_robot.controller.VirtualField;
import virtual_robot.util.AngleUtils;

/**
 * Specialized RevoluteJoint that can be set at a specified angle (in specified units),
 * with some position tolerance.
 */
public class Hinge extends RevoluteJoint {

    private double maxTorque = 100; // Newton-meters
    private double speed = 10;  // radians per sec
    private double tolerance = 0.005;  // radians
    private double positionRadians = 0;
    private VirtualField.Unit defaultUnit;

    /**
     * Constructor
     * @param b1        First Body
     * @param b2        Second Body
     * @param anchor    Anchor Point, WORLD Coordinates (in units specified by unit)
     * @param unit      Unit for specifying the Anchor Point
     */
    public Hinge(Body b1, Body b2, Vector2 anchor, VirtualField.Unit unit){
        super(b1, b2, anchor.product(VirtualField.conversionFactor(unit, VirtualField.Unit.METER)));
        defaultUnit = unit;
        this.setMotorEnabled(true);
        this.setMaximumMotorTorque(maxTorque);
        this.setLimitEnabled(true);
        this.setReferenceAngle(0);
        this.setLimits(-tolerance, tolerance);
        this.setMotorSpeed(speed);
    }


    /**
     * Set slide position, in radians, relative to the position at the time of
     * Hinge creation.
     */
    public void setPosition(double positionRadians){
        double motorDir = Math.signum(AngleUtils.normalizeRadians(positionRadians - this.positionRadians));
        this.positionRadians = AngleUtils.normalizeRadians(positionRadians);
        this.setReferenceAngle(this.positionRadians);
        this.setLimits(-tolerance, tolerance);
        this.setMotorSpeed(speed * motorDir);
    }

    /**
     * Get current Hinge position in default radians, relative to the position at the time of
     * Slide creation.
     * @return
     */
    public double getPosition(){
        return positionRadians;
    }


}
