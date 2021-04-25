package virtual_robot.controller.game_elements.classes;

import javafx.scene.Group;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.MassType;
import virtual_robot.config.UltimateGoal;
import virtual_robot.controller.*;
import virtual_robot.dyn4j.VRBody;

import java.util.ArrayList;
import java.util.List;

@GameElementConfig(name = "Wobble Goal", filename = "wobble_goal", forGame = UltimateGoal.class, numInstances = 2)
public class WobbleGoal extends VirtualGameElement {

    public static final double RADIUS_INCHES = 4.0;
    public static final List<WobbleGoal> wobbles = new ArrayList<>();

    private VirtualBot bot;
    VRBody wobbleBody = null;
    BodyFixture wobbleFixture;

    public static final long WOBBLE_CATEGORY = 512;
    public static final CategoryFilter WOBBLE_FILTER = new CategoryFilter(WOBBLE_CATEGORY, Filters.MASK_ALL);

    private boolean onField = false;

    public void initialize(){
        super.initialize();
    }

    @Override
    public void setUpDisplayGroup(Group group) {
        super.setUpDisplayGroup(group);
    }

    @Override
    public void updateState(double millis) {
        x = wobbleBody.getTransform().getTranslationX() * FIELD.PIXELS_PER_METER;
        y = wobbleBody.getTransform().getTranslationY() * FIELD.PIXELS_PER_METER;
        headingRadians = wobbleBody.getTransform().getRotationAngle();
    }

    @Override
    public synchronized void updateDisplay() {
        super.updateDisplay();
    }

    @Override
    public void setUpBody(){
        elementBody = new VRBody(this);
        wobbleBody = elementBody;
        double wobbleRadiusMeters = 4.0 / VirtualField.INCHES_PER_METER;
        wobbleFixture = wobbleBody.addFixture(
                new org.dyn4j.geometry.Circle(wobbleRadiusMeters), 1, 0, 0);
        wobbleFixture.setFilter(WOBBLE_FILTER);
        wobbleBody.setMass(MassType.NORMAL);
        wobbleBody.setLinearDamping(100.0);
    }

    /**
     * Add or remove the wobble to/from field, which includes the following actions:
     *   1) set the value of onField
     *   2) Remove body from world, or add body to world.
     *   3) Remove or add the element displayGroup from/to the display.
     *
     * @param onField
     */
    public void setOnField(boolean onField){
        this.onField = onField;
        if (onField && !world.containsBody(wobbleBody)) world.addBody(wobbleBody);
        else if (!onField && world.containsBody(wobbleBody)) world.removeBody(wobbleBody);
        if (onField) addToDisplay();
        else removeFromDisplay();
    }

    public boolean isOnField(){
        return this.onField;
    }

    @Override
    public void stop(){
        wobbleBody.setLinearVelocity(0,0);
        wobbleBody.setAngularVelocity(0);
    }
}
