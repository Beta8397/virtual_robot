package virtual_robot.game_elements.classes;

import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.shape.Circle;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.joint.WeldJoint;
import org.dyn4j.geometry.Vector2;
import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;
import virtual_robot.games.FreightFrenzy;
import virtual_robot.util.AngleUtils;

import java.util.ArrayList;
import java.util.List;

@GameElementConfig(name = "Carousel", filename = "carousel", forGame = FreightFrenzy.class, numInstances = 2)
public class Carousel extends VirtualGameElement {

    public static final double CAROUSEL_RADIUS = 7.5;
    public static final List<Carousel> carousels = new ArrayList<>();
    private static Carousel redCarousel = null;
    private static Carousel blueCarousel = null;

    Body carouselBody = null;

    // Category and filter for collisions
    public static final long CAROUSEL_CATEGORY = 4096;
    public static final CategoryFilter CAROUSEL_FILTER = new CategoryFilter(CAROUSEL_CATEGORY, CAROUSEL_CATEGORY);

    // Outer circle from the .fxml file; will use to generate dyn4j Body
    @FXML private Circle outerCircle;

    private DuckFreight duckToAttach = null;
    private DuckFreight attachedDuck = null;
    private WeldJoint duckJoint = null;
    private double headingOnAttach = 0;

    public void initialize(){
        super.initialize();
        if (redCarousel == null) redCarousel = this;
        else blueCarousel = this;
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

        if (duckToAttach != null && attachedDuck == null) {
            attachedDuck = duckToAttach;
        }

        handleDuckDetach();
        handleDuckAttach();
        duckToAttach = null;
    }

    @Override
    public synchronized void updateDisplay() {
        super.updateDisplay();
        displayGroup.toFront();
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
                new FixtureData(CAROUSEL_FILTER, 1, 0, 0, 1.1, 1.1));
        carouselBody = elementBody;       // Alias for elementBody
    }

    public DuckFreight getDuckToAttach() { return attachedDuck; }

    public void setDuckToAttach(DuckFreight duck) { duckToAttach = duck; }

    public DuckFreight getAttachedDuck() { return attachedDuck; }

    private void handleDuckAttach(){
        if (duckToAttach == null || attachedDuck != null) return;
        attachedDuck = duckToAttach;
        DuckFreight.ducksOffFieldRed.remove(attachedDuck);
        DuckFreight.ducksOffFieldBlue.remove(attachedDuck);
        attachedDuck.setOnField(false);
        Vector2 anchor = this == redCarousel?
                new Vector2(72.0 / VirtualField.INCHES_PER_METER, -67.0 / VirtualField.INCHES_PER_METER)
                : new Vector2(-72.0 / VirtualField.INCHES_PER_METER, -67.0 / VirtualField.INCHES_PER_METER);
        attachedDuck.setLocationMeters(anchor);
        if (duckJoint != null) world.removeJoint(duckJoint);
        duckJoint = new WeldJoint(attachedDuck.getElementBody(), this.getElementBody(), anchor);
        attachedDuck.setCategoryFilter(Freight.ON_CAROUSEL_FILTER);
        attachedDuck.setOnField(true);
        world.addJoint(duckJoint);
        headingOnAttach = getHeadingRadians();
    }

    private void handleDuckDetach(){
        // Can only detach a duck if there is already one attached
        if (attachedDuck == null) return;

        // Whether to release/eject an attached duck depends on how much the carousel has turned since duck was attached
        double heading = getHeadingRadians();
        double headingChange = AngleUtils.normalizeRadians(heading - headingOnAttach);
        boolean releaseToField = false;
        boolean eject = false;
        if (this == redCarousel){
            releaseToField = headingChange < Math.toRadians(60) && headingChange > Math.toRadians(30);
            eject = headingChange < Math.toRadians(30) && headingChange > Math.toRadians(10);
        } else {
            releaseToField = headingChange > Math.toRadians(-60) && headingChange < Math.toRadians(-30);
            eject = headingChange > Math.toRadians(-30) && headingChange < Math.toRadians(-10);
        }

        //If we aren't releasing or ejecting, stop here
        if (!releaseToField && !eject) return;

        //Now we are either releasing duck to the field or ejecting it off the field
        DuckFreight releasedDuck = attachedDuck;
        attachedDuck = null;
        releasedDuck.setOnField(false);
        world.removeJoint(duckJoint);
        duckJoint = null;

        // If ejecting, add duck back to the bucket of available ducks for red or blue, then stop
        if (eject){
            if (this == redCarousel && !DuckFreight.ducksOffFieldRed.contains(releasedDuck)) {
                DuckFreight.ducksOffFieldRed.add(releasedDuck);
            } else if (this == blueCarousel && !DuckFreight.ducksOffFieldBlue.contains(releasedDuck)){
                DuckFreight.ducksOffFieldBlue.add(releasedDuck);
            }
            return;
        }

        // Here we know we're releasing a duck to the field. Reposition it (depends on red vs blue), set a velocity,
        // give it a new CollisionFilter, and add it back to field.

        Vector2 duckPosition = this == redCarousel?
                new Vector2(68.0 / VirtualField.INCHES_PER_METER, -62.0 / VirtualField.INCHES_PER_METER) :
                new Vector2(-68.0 / VirtualField.INCHES_PER_METER, -62.0 / VirtualField.INCHES_PER_METER);
        releasedDuck.setOnField(true);
        releasedDuck.setCategoryFilter(Freight.NORMAL_FILTER);
        releasedDuck.getElementBody().getTransform().setTranslation(duckPosition);
        releasedDuck.getElementBody().setLinearVelocity(0, 0.05);

    }

}
