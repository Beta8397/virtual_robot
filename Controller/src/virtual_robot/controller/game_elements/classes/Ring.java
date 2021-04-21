package virtual_robot.controller.game_elements.classes;

import javafx.scene.Group;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.ContactCollisionData;
import org.dyn4j.world.NarrowphaseCollisionData;
import org.dyn4j.world.listener.CollisionListenerAdapter;
import org.dyn4j.world.listener.ContactListenerAdapter;
import virtual_robot.config.GameObject;
import virtual_robot.config.UltimateGoal;
import virtual_robot.controller.*;

import java.util.Random;

@GameElementConfig(name = "Ring", filename = "ring", forGame = UltimateGoal.class, numInstances = 10)
public class Ring extends VirtualGameElement {
    public static final double RING_RADIUS_INCHES = 2.5;
    private boolean onField = true;
    private boolean inFlight = false;
    private boolean rolling = false;
    private boolean controlled = false;
    private boolean stacked = false;

    VRBody ringBody;
    BodyFixture ringFixture;

    public static long RING_CATEGORY = 1024;
    public static final CategoryFilter RING_FILTER = new CategoryFilter(RING_CATEGORY, Filters.MASK_ALL);
    public static final CategoryFilter RING_FLYING_FILTER = new CategoryFilter(RING_CATEGORY,
            Filters.MASK_ALL & ~Filters.WALL);
    public static final CategoryFilter RING_STACKED_FILTER = new CategoryFilter(RING_CATEGORY,
            Filters.MASK_ALL & ~RING_CATEGORY);

    public void initialize(){
        super.initialize();
    }

    @Override
    public void setUpDisplayGroup(Group group) {
        super.setUpDisplayGroup(group);
    }

    @Override
    public synchronized void updateState(double millis) {
        x = ringBody.getTransform().getTranslationX() * FIELD.PIXELS_PER_METER;
        y = ringBody.getTransform().getTranslationY() * FIELD.PIXELS_PER_METER;
        headingRadians = ringBody.getTransform().getRotationAngle();
    }

    @Override
    public synchronized void updateDisplay() {
        displayGroup.setVisible(onField);
        super.updateDisplay();
    }

    @Override
    public void setUpBody(){
        elementBody = new VRBody(this);
        ringBody = elementBody;
        double ringRadiusMeters = RING_RADIUS_INCHES / VirtualField.INCHES_PER_METER;
        ringFixture = ringBody.addFixture(
                new org.dyn4j.geometry.Circle(ringRadiusMeters), 1, 0, 0);
        ringFixture.setFilter(RING_FILTER);
        ringBody.setMass(MassType.NORMAL);
        ringBody.setLinearDamping(100);
    }

    public VRBody getRingBody(){ return ringBody; }

    public boolean isInFlight() {
        return inFlight;
    }

    public void setInFlight(boolean inFlight) {
        this.inFlight = inFlight;
        ringFixture.setFilter(inFlight? RING_FLYING_FILTER : RING_FILTER);
        ringBody.setLinearDamping(0);
    }

    public boolean isRolling() {
        return rolling;
    }

    public void setRolling(boolean rolling) {
        this.rolling = rolling;
        ringBody.setLinearDamping(rolling? 0.2 : 100.0);
    }

    public void setControlled(boolean controlled) { this.controlled = controlled; }

    public boolean getControlled() { return controlled; }

    public void setStacked(boolean stacked){
        this.stacked = stacked;
        ringFixture.setFilter(stacked? RING_STACKED_FILTER : RING_FILTER);
    }

    public boolean isStacked() { return this.stacked; }

    public void setVelocityInchesPerSec(double vx, double vy) {
        this.ringBody.setLinearVelocity(vx / VirtualField.INCHES_PER_METER, vy / VirtualField.INCHES_PER_METER);
    }

    public void setVelocityMetersPerSec(double vx, double vy){
        this.ringBody.setLinearVelocity(vx, vy);
    }

}
