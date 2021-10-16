package virtual_robot.game_elements.classes;

import com.qualcomm.robotcore.util.ElapsedTime;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.shape.Circle;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.joint.RevoluteJoint;
import org.dyn4j.dynamics.joint.WeldJoint;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import virtual_robot.controller.Filters;
import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;
import virtual_robot.games.FreightFrenzy;
import virtual_robot.util.AngleUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

@GameElementConfig(name = "Carousel", filename = "carousel", forGame = FreightFrenzy.class, numInstances = 2)
public class Carousel extends VirtualGameElement {

    private Random random = new Random();

    public static final double CAROUSEL_RADIUS = 7.5;
    public static final List<Carousel> carousels = new ArrayList<>();
    private static Carousel redCarousel = null;
    private static Carousel blueCarousel = null;

    Body carouselBody = null;
    Body anchorBody = null;
    RevoluteJoint anchorJoint = null;

    // Category and filter for collisions
    public static final long CAROUSEL_CATEGORY = 4096;
    public static final long CAROUSEL_SPINNER_CATEGORY = 8192;
    public static final CategoryFilter CAROUSEL_FILTER =
            new CategoryFilter(CAROUSEL_CATEGORY, CAROUSEL_SPINNER_CATEGORY);
    public static final CategoryFilter ANCHOR_FILTER =
            new CategoryFilter(CAROUSEL_CATEGORY, CAROUSEL_SPINNER_CATEGORY | Filters.CHASSIS);

    // Outer circle from the .fxml file; will use to generate dyn4j Body
    @FXML private Circle outerCircle;

    private DuckFreight duckToAttach = null;
    private DuckFreight attachedDuck = null;
    private WeldJoint duckJoint = null;
    private double headingOnAttach = 0;

    // Timer to determine the interval since a duck was either released or ejected
    ElapsedTime timer = new ElapsedTime();

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
     * Set up the dyn4j physics bodys for the carousel.
     */
    @Override
    public void setUpBody(){
        /*
         * Use Dyn4jUtil.createBody to create a Body. outerCircle (from the .fxml file) is used to generate
         * two bodies: The carousel body and the anchor body. The anchor body has infinite mass (and is thus
         * immobile). The carousel body is attached to the anchor body by a revolute joint, so it can rotate
         * around its center, but cannot translate.
         */
        elementBody = Dyn4jUtil.createBody(outerCircle, this, 0, 0,
                new FixtureData(CAROUSEL_FILTER, 1, 0, 1.0, 1.05, 1.05));
        carouselBody = elementBody;       // Alias for elementBody
        anchorBody = Dyn4jUtil.createBody(outerCircle, this, 0, 0,
                new FixtureData(ANCHOR_FILTER, 1, 0, 0));
        anchorBody.setMass(MassType.INFINITE);
        anchorJoint = new RevoluteJoint(carouselBody, anchorBody, new Vector2(0,0));
        carouselBody.setAngularDamping(1.0);
    }

    /**
     * Set the location of the carousel on the field. This requires placement of
     * both the carousel body and the anchor body at the requested position.
     *
     * @param xPixels
     * @param yPixels
     */
    @Override
    public void setLocation(double xPixels, double yPixels){
        this.x = xPixels;
        this.y = yPixels;
        Transform tAnchor = new Transform();
        tAnchor.setTranslation(x / VirtualField.PIXELS_PER_METER, y / VirtualField.PIXELS_PER_METER);
        tAnchor.setRotation(headingRadians);
        anchorBody.setTransform(tAnchor);
        anchorBody.setLinearVelocity(0,0);
        anchorBody.setAngularVelocity(0);
        anchorBody.clearAccumulatedForce();
        anchorBody.clearAccumulatedTorque();
        Transform tCarousel = new Transform();
        tCarousel.setTranslation(x / VirtualField.PIXELS_PER_METER, y / VirtualField.PIXELS_PER_METER);
        tCarousel.setRotation(headingRadians);
        carouselBody.setTransform(tCarousel);
        carouselBody.setLinearVelocity(0,0);
        carouselBody.setAngularVelocity(0);
        carouselBody.clearAccumulatedForce();
        carouselBody.clearAccumulatedTorque();
    }

    /**
     * Set the Carousel on the field (onField = true), or remove it (onField = false).
     * This requires addition or removal of the carousel body, anchor body, and anchor joint from the world.
     * @param onField
     */
    @Override
    public void setOnField(boolean onField){
        this.onField = onField;
        if (onField){
            if (!world.containsBody(carouselBody)) world.addBody(carouselBody);
            if (!world.containsBody(anchorBody)) world.addBody(anchorBody);
            if (!world.containsJoint(anchorJoint)) world.addJoint(anchorJoint);
        }else {
            if (world.containsBody(carouselBody)) world.removeBody(carouselBody);
            if (world.containsBody(anchorBody)) world.removeBody(anchorBody);
            if (world.containsJoint(anchorJoint)) world.removeJoint(anchorJoint);
        }
        if (onField) addToDisplay();
        else removeFromDisplay();
    }

    public DuckFreight getDuckToAttach() { return duckToAttach; }

    public void setDuckToAttach(DuckFreight duck) {
        duckToAttach = duck;
        if (this == redCarousel) {
            System.out.println("SetDuckToAttach: duckToAttach = " + duckToAttach);
        }
    }

    public DuckFreight getAttachedDuck() { return attachedDuck; }

    public void handleDuckAttach(){
        if (this == redCarousel){
            System.out.println("HandleDuckAttach: dta = " + duckToAttach);
        }
        if (duckToAttach == null || attachedDuck != null) return;
        attachedDuck = duckToAttach;
        DuckFreight.ducksOffFieldRed.remove(attachedDuck);
        DuckFreight.ducksOffFieldBlue.remove(attachedDuck);
        attachedDuck.setOnField(false);
        Vector2 anchor = this == redCarousel?
                new Vector2(71.0 / VirtualField.INCHES_PER_METER, -66.0 / VirtualField.INCHES_PER_METER)
                : new Vector2(-71.0 / VirtualField.INCHES_PER_METER, -66.0 / VirtualField.INCHES_PER_METER);
        attachedDuck.setLocationMeters(anchor);
        if (duckJoint != null) world.removeJoint(duckJoint);
        duckJoint = new WeldJoint(attachedDuck.getElementBody(), this.getElementBody(), anchor);
        attachedDuck.setCategoryFilter(Freight.ON_CAROUSEL_FILTER);
        attachedDuck.setOnField(true);
        world.addJoint(duckJoint);
        headingOnAttach = getHeadingRadians();
        if (this == redCarousel) {
            System.out.println();
            System.out.println("Just attached duck. Heading on attach = " + Math.toDegrees(headingOnAttach));
        }
    }

    public void clearAttachedDuck(){
        if (attachedDuck != null) {
            if (world.containsBody(attachedDuck.getElementBody())) world.removeBody(attachedDuck.getElementBody());
            if (world.containsJoint(duckJoint)) world.removeJoint(duckJoint);
            attachedDuck = null;
            duckJoint = null;
        }
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
        if (!releaseToField && !eject) {
            if (this == redCarousel) {
                System.out.println("Duck remains attached. Heading = " + Math.toDegrees(heading)
                        + "  headingChange = " + Math.toDegrees(headingChange) + "  Release: " + releaseToField);
            }
            return;
        }

        if (this == redCarousel){
            System.out.println("Detaching duck");
        }

        //Now we are either releasing duck to the field or ejecting it off the field
        DuckFreight releasedDuck = attachedDuck;
        attachedDuck = null;
        releasedDuck.setOnField(false);
        world.removeJoint(duckJoint);
        duckJoint = null;
        timer.reset();

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
                new Vector2(68.0 / VirtualField.INCHES_PER_METER,
                        (-60.0 + 20*random.nextDouble()) / VirtualField.INCHES_PER_METER) :
                new Vector2(-68.0 / VirtualField.INCHES_PER_METER,
                        (-60.0 + 20*random.nextDouble()) / VirtualField.INCHES_PER_METER );
        releasedDuck.setOnField(true);
        releasedDuck.setCategoryFilter(Freight.NORMAL_FILTER);
        releasedDuck.getElementBody().getTransform().setTranslation(duckPosition);
        releasedDuck.getElementBody().setLinearVelocity(0, 0.2);
    }

    public double getTimerMilliseconds(){
        return timer.milliseconds();
    }

}
