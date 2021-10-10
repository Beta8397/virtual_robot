package virtual_robot.game_elements.classes;

import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Shape;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.Body;
import virtual_robot.games.UltimateGoal;
import virtual_robot.controller.*;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

@GameElementConfig(name = "Wobble Goal", filename = "wobble_goal", forGame = UltimateGoal.class, numInstances = 2)
public class WobbleGoal extends VirtualGameElement {

    public static final double RADIUS_INCHES = 4.0;
    public static final List<WobbleGoal> wobbles = new ArrayList<>();

    Body wobbleBody = null;

    // Category and filter for collisions
    public static final long WOBBLE_CATEGORY = 512;
    public static final long WOBBLE_HANDLE_CATEGORY = 1024;
    public static final CategoryFilter WOBBLE_FILTER = new CategoryFilter(WOBBLE_CATEGORY, Filters.MASK_ALL);
    public static final CategoryFilter WOBBLE_HANDLE_FILTER =
            new CategoryFilter(WOBBLE_HANDLE_CATEGORY, Filters.ARM);

    // Outer circle from the .fxml file; will use to generate dyn4j Body
    @FXML private Circle outerCircle;
    @FXML private Circle innerCircle;

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
//        elementBody = Dyn4jUtil.createBody(outerCircle, this, 0, 0,
//                new FixtureData(WOBBLE_FILTER, 1, 0, 0));
        HashMap<Shape, FixtureData> map = new HashMap<>();
        map.put(outerCircle, new FixtureData(WOBBLE_FILTER, 1, 0, 0.25, false));
        map.put(innerCircle, new FixtureData(WOBBLE_HANDLE_FILTER, 1, 0, 0.25, false, 2.0, 2.0));
        elementBody = Dyn4jUtil.createBody(displayGroup, this, 0, 0, map);

        wobbleBody = elementBody;       // Alias for elementBody
        wobbleBody.setLinearDamping(100.0);     // Lots of damping (simulates floor-wobble friction)
    }

}
