package virtual_robot.controller.game_elements.classes;

import javafx.scene.Group;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.MassType;
import virtual_robot.config.UltimateGoal;
import virtual_robot.controller.*;

@GameElementConfig(name = "Ring", filename = "ring", forGame = UltimateGoal.class, numInstances = 10)
public class Ring extends VirtualGameElement {
    public static final double RING_RADIUS_INCHES = 2.5;
    private boolean onField = true;
    private boolean inFlight = false;
    private boolean rolling = false;
    private VirtualBot bot = null;

    VRBody ringBody;
    BodyFixture ringFixture;

    public static long RING_CATEGORY = 1024;
    public static CategoryFilter RING_FILTER = new CategoryFilter(RING_CATEGORY, Filters.MASK_ALL);

    @Override
    protected void setUpDisplayGroup(Group group) {
        super.setUpDisplayGroup(group);
    }

    @Override
    public synchronized void updateState(double millis) {
        x += ringBody.getTransform().getTranslationX() * FIELD.PIXELS_PER_METER;
        y += ringBody.getTransform().getTranslationY() * FIELD.PIXELS_PER_METER;
        headingRadians = ringBody.getTransform().getRotationAngle();
    }

    @Override
    public synchronized void updateDisplay() {
        if (!onField) {
            y = VirtualField.getInstance().Y_MAX + 15; // move ring off the field
        }
        super.updateDisplay();
    }

    @Override
    public VirtualBot getControlledBy() {
        return bot;
    }

    @Override
    public void setControlledBy(VirtualBot bot) {
        this.bot = bot;
    }

    @Override
    public void setUpPhysicsBodies(){
        ringBody = new VRBody(this);
        double ringRadiusMeters = RING_RADIUS_INCHES / VirtualField.INCHES_PER_METER;
        ringFixture = ringBody.addFixture(
                new org.dyn4j.geometry.Circle(ringRadiusMeters), 1, 0, 0);
        ringFixture.setFilter(RING_FILTER);
        ringBody.setMass(MassType.NORMAL);
        world.addBody(ringBody);
    }

    public boolean isOnField() {
        return onField;
    }

    public void setOnField(boolean onField) {
        this.onField = onField;
    }

    public boolean isInFlight() {
        return inFlight;
    }

    public void setInFlight(boolean inFlight) {
        this.inFlight = inFlight;
    }

    public boolean isRolling() {
        return rolling;
    }

    public void setRolling(boolean rolling) {
        this.rolling = rolling;
    }
}
