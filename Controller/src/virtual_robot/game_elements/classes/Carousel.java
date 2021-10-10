package virtual_robot.game_elements.classes;

import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.shape.Circle;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.Body;
import virtual_robot.controller.Filters;
import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;
import virtual_robot.games.FreightFrenzy;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

//@GameElementConfig(name = "Carousel", filename = "carousel", forGame = FreightFrenzy.class, numInstances = 2)
public class Carousel extends VirtualGameElement {

    public static final double CAROUSEL_RADIUS = 7.5;
    public static final List<Carousel> carousels = new ArrayList<>();

    Body carouselBody = null;

    // Category and filter for collisions
    public static final long CAROUSEL_CATEGORY = 4096;
    public static final CategoryFilter CAROUSEL_FILTER = new CategoryFilter(CAROUSEL_CATEGORY, CAROUSEL_CATEGORY);

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
        x = carouselBody.getTransform().getTranslationX() * VirtualField.PIXELS_PER_METER;
        y = carouselBody.getTransform().getTranslationY() * VirtualField.PIXELS_PER_METER;
        headingRadians = carouselBody.getTransform().getRotationAngle();
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
                new FixtureData(CAROUSEL_FILTER, 1, 0, 0));

        carouselBody = elementBody;       // Alias for elementBody
        carouselBody.setLinearDamping(100.0);     // Lots of damping (simulates floor-wobble friction)
    }

    public Circle getOuterCircle(){ return outerCircle; }
}
