package virtual_robot.game_elements.classes;

import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.shape.Circle;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import virtual_robot.games.UltimateGoal;
import virtual_robot.controller.*;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;

import java.util.ArrayList;
import java.util.List;

@GameElementConfig(name = "Ring", filename = "ring", forGame = UltimateGoal.class, numInstances = 10)
public class Ring extends VirtualGameElement {

    public static final double RING_RADIUS_INCHES = 2.5;
    public static final List<Ring> rings = new ArrayList<>();
    public static final List<Ring> ringsOffField = new ArrayList<>();

    // Reference to outer circle from the .fxml file, to be used when generating the dyn4j Body
    @FXML
    private Circle outerCircle;

    /*
     * The STATUS of a Ring determines whether it is "in play", its linear damping, and its collision filter.
     */
    public enum RingStatus {
        NORMAL(true, 100, RING_FILTER),
        ROLLING(true, 0.05, RING_FILTER),
        FLYING(true, 0, RING_FLYING_FILTER),
        STACKED(true, 100, RING_STACKED_FILTER),
        CONTROLLED(false, 0, RING_FILTER),
        OFF_FIELD(false, 0, RING_FILTER);   //Only these are available to human player

        private boolean inPlay;             // Is ring controlled by dyn4j physics world?
        private double linearDamping;       // effectively, the friction between floor and world
        private CategoryFilter filter;      // Filter for collisions between ring and other bodies

        RingStatus(boolean inPlay, double linearDamping, CategoryFilter filter){
            this.inPlay = inPlay;
            this.linearDamping = linearDamping;
            this.filter = filter;
        }

        public boolean isInPlay() { return inPlay; }
        public double getLinearDamping() { return linearDamping; }
        public CategoryFilter getFilter() { return filter; }
    }

    /*
     * The setStatus method will remove or add the ringBody from/to the dyn4j world, as appropriate.
     * For that reason, it should not be called from a collision listener (which is called during a world
     * update).
     *
     * Instead, a collision listener should call the setNextRingStatus method; the call to setRingStatus can
     * then be delayed until the next call to updateStateAndSensors.
     */
    private RingStatus status = RingStatus.OFF_FIELD;
    private RingStatus nextStatus = RingStatus.OFF_FIELD;

    private double RING_BOUNDARY;   // TODO: make this final and static (and figure out how to initialize)

    /*
     * Physics body and BodyFixture for the ring. ringBody will just be an alias of elementBody from
     * the VirtualGameElement parent class.
     */
    Body ringBody;
    BodyFixture ringFixture;

    //The bit indicating the category of rings for collision detection (0x10000000000)
    public static long RING_CATEGORY = 2048;

    //Standard ring filter: collide with everything that is allowed to collide with ring
    public static final CategoryFilter RING_FILTER = new CategoryFilter(RING_CATEGORY, Filters.MASK_ALL);

    //Flying ring filter: collide with NOTHING
    public static final CategoryFilter RING_FLYING_FILTER = new CategoryFilter(RING_CATEGORY, 0);

    //Stacked ring filter: collide with everything except ring (to prevent stack from becoming unstable)
    public static final CategoryFilter RING_STACKED_FILTER = new CategoryFilter(RING_CATEGORY,
            Filters.MASK_ALL & ~RING_CATEGORY);

    public void initialize(){
        super.initialize();

        /*
         * When the ring passes beyond this boundary, it's status will be changed to OFF_FIELD,
         * to allow collection by the human player.
         */

        RING_BOUNDARY = VirtualField.HALF_FIELD_WIDTH + 4 * RING_RADIUS_INCHES * VirtualField.PIXELS_PER_INCH;
    }

    @Override
    public void setUpDisplayGroup(Group group) {
        super.setUpDisplayGroup(group);
    }

    @Override
    public synchronized void updateState(double millis) {

        /*
         * Obtain the location and heading of the ring from the dyn4j body,
         * and convert to pixels for subsequent display.
         */
        x = ringBody.getTransform().getTranslationX() * VirtualField.PIXELS_PER_METER;
        y = ringBody.getTransform().getTranslationY() * VirtualField.PIXELS_PER_METER;
        headingRadians = ringBody.getTransform().getRotationAngle();

        /*
         * Check whether ring is off the field, and change nextStatus if appropriate.
         * This will override any changes made to nextStatus within collision Listeners
         * during the previous update of the dyn4j world.
         */
        if (Math.abs(x) > RING_BOUNDARY || Math.abs(y) > RING_BOUNDARY){
            nextStatus = RingStatus.OFF_FIELD;
        }

        /*
         * Change to next ring status (which in the large majority of iterations will be the same as the
         * current status). nextStatus can be changed when the ring goes off the field, and also during
         * calls to collision Listeners.
         */
        setStatus(nextStatus);
    }

    /*
     * No special display handling for Ring. In fact, don't really need to override the parent method.
     */
    @Override
    public synchronized void updateDisplay() {
        super.updateDisplay();
    }

    /**
      * Instantiate elementBody, and set its parent to this ring. Also save this in an alias: ringBody.
      * Add a fixture with circular shape to the body. Initial status: OFF_FIELD.
      */
    @Override
    public void setUpBody(){
        /*
         * Use Dyn4jUtil.createBody to generate and configure a dyn4j Body, using the outerCircle from the .fxml
         * file. Note that we save a reference to the BodyFixture. this is needed so that the collision
         * filter can be changed as ring status changes.
         */
        elementBody = Dyn4jUtil.createBody(outerCircle,this, 0, 0,
                new FixtureData(RING_FILTER, 1.0, 0, 0));
        ringBody = elementBody;     //Just an alias
        ringFixture = ringBody.getFixture(0);
        this.setStatus(RingStatus.OFF_FIELD);
    }

    /**
     * Set velocity of the Ring, in inches per second.
     * @param vx
     * @param vy
     */
    public void setVelocityInchesPerSec(double vx, double vy) {
        this.ringBody.setLinearVelocity(vx / VirtualField.INCHES_PER_METER, vy / VirtualField.INCHES_PER_METER);
    }

    /**
     * Set the velocity of the Ring, in meters per second.
     * @param vx
     * @param vy
     */
    public void setVelocityMetersPerSec(double vx, double vy){
        this.ringBody.setLinearVelocity(vx, vy);
    }

    /**
     * Get next ring status
     * @return
     */
    public RingStatus getNextStatus(){ return nextStatus; }

    /**
     * Set the next ring status. Use this method (instead of the setStatus method) when changing ring
     * status from Listeners, as the setStatus method makes changes in the physics world that will
     * cause problems if called during the world update.
     * @param nextStatus
     */
    public void setNextStatus(RingStatus nextStatus) { this.nextStatus = nextStatus; }

    /**
     * Get current ring status
     * @return
     */
    public RingStatus getStatus() { return status; }

    /**
     * Set ring status.
     * Do not call this method from collision listeners (use setNextRingStatus instead).
     * @param status
     */
    public void setStatus(RingStatus status){
        // Update both the status and nextStatus fields to match the status argument
        this.status = status;
        this.nextStatus = status;
        // Add/remove to/from the physics world and the display, depending on the value of inPlay for the new status.
        setOnField(status.isInPlay());
        // Update the ringsOffField list if necessary (it keeps track of which rings are available to human player)
        if (status == RingStatus.OFF_FIELD && !ringsOffField.contains(this)) {
            ringsOffField.add(this);
        } else if (status != RingStatus.OFF_FIELD && ringsOffField.contains(this)){
            ringsOffField.remove(this);
        }
        // Set the linearDamping and the collision filter corresponding to the new status.
        ringBody.setLinearDamping(status.getLinearDamping());
        ringFixture.setFilter(status.getFilter());
    }

}
