package virtual_robot.game_elements.classes;

import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.shape.Circle;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.MassType;
import virtual_robot.config.UltimateGoal;
import virtual_robot.controller.*;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;

import java.util.ArrayList;
import java.util.List;

@GameElementConfig(name = "Wobble Goal", filename = "wobble_goal", forGame = UltimateGoal.class, numInstances = 2)
public class WobbleGoal extends VirtualGameElement {

    public static final double RADIUS_INCHES = 4.0;
    public static final List<WobbleGoal> wobbles = new ArrayList<>();

    Body wobbleBody = null;

    // Category and filter for collisions
    public static final long WOBBLE_CATEGORY = 512;
    public static final CategoryFilter WOBBLE_FILTER = new CategoryFilter(WOBBLE_CATEGORY, Filters.MASK_ALL);

    private boolean onField = false;

    // Outer circle from the .fxml file; will use to generate dyn4j Body
    @FXML
    private Circle outerCircle;

    public void initialize(){
        super.initialize();
    }

    @Override
    public void setUpDisplayGroup(Group group) {
        super.setUpDisplayGroup(group);
    }

    /**
     * The implementation of updateState for WobbleGoal is simple: just obtain (x, y, headingRadians)
     * from the physics body, and translate x and y from meters to pixels.
     * @param millis milliseconds since the previous update
     */
    @Override
    public void updateState(double millis) {
        x = wobbleBody.getTransform().getTranslationX() * VirtualField.PIXELS_PER_METER;
        y = wobbleBody.getTransform().getTranslationY() * VirtualField.PIXELS_PER_METER;
        headingRadians = wobbleBody.getTransform().getRotationAngle();
    }

    @Override
    public synchronized void updateDisplay() {
        super.updateDisplay();
    }

    /**
     * Set up the dyn4j physics body for the wobble goal.
     */
    @Override
    public void setUpBody(){
        /*
         * Use Dyn4jUtil.createBody to create a Body. outerCircle (from the .fxml file) is used to generate
         * a single BodyFixture that is added to the Body.
         */
        elementBody = Dyn4jUtil.createBody(outerCircle, this, 0, 0,
                new FixtureData(WOBBLE_FILTER, 1, 0, 0));
        wobbleBody = elementBody;       // Alias for elementBody
        wobbleBody.setLinearDamping(100.0);     // Lots of damping (simulates floor-wobble friction)
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
