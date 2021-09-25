package virtual_robot.game_elements.classes;

import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Shape;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.Body;
import virtual_robot.controller.Filters;
import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;
import virtual_robot.games.FreightFrenzy;
import virtual_robot.games.UltimateGoal;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

@GameElementConfig(name = "Shipping Hub", filename = "shipping_hub", forGame = FreightFrenzy.class, numInstances = 3)
public class ShippingHub extends VirtualGameElement {

    public static final List<ShippingHub> shippingHubs = new ArrayList<>();

    private boolean onField = false;

    Body shippingHubBody = null;

    // Category and filter for collisions
    public static final long HUB_CATEGORY = 512;
    public static final CategoryFilter HUB_FILTER = new CategoryFilter(HUB_CATEGORY, Filters.MASK_ALL);

    // Outer circle from the .fxml file; will use to generate dyn4j Body
    @FXML private Circle outerCircle;

    public void initialize(){
        super.initialize();
    }

    @Override
    public void setUpDisplayGroup(Group group) {
        super.setUpDisplayGroup(group);
    }

    /**
     * The implementation of updateState for ShippingHub is simple: just obtain (x, y, headingRadians)
     * from the physics body, and translate x and y from meters to pixels.
     * @param millis milliseconds since the previous update
     */
    @Override
    public void updateState(double millis) {
        x = shippingHubBody.getTransform().getTranslationX() * VirtualField.PIXELS_PER_METER;
        y = shippingHubBody.getTransform().getTranslationY() * VirtualField.PIXELS_PER_METER;
        headingRadians = shippingHubBody.getTransform().getRotationAngle();
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
                new FixtureData(HUB_FILTER, 1, 0, 0));

        shippingHubBody = elementBody;       // Alias for elementBody
        shippingHubBody.setLinearDamping(100.0);     // Lots of damping (simulates floor-wobble friction)
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
        if (onField && !world.containsBody(shippingHubBody)) world.addBody(shippingHubBody);
        else if (!onField && world.containsBody(shippingHubBody)) world.removeBody(shippingHubBody);
        if (onField) addToDisplay();
        else removeFromDisplay();
    }

    public boolean isOnField(){
        return this.onField;
    }

    @Override
    public void stop(){
        shippingHubBody.setLinearVelocity(0,0);
        shippingHubBody.setAngularVelocity(0);
    }

    public Circle getOuterCircle(){ return outerCircle; }
}
