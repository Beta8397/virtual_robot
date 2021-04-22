package virtual_robot.controller.game_elements.classes;

import javafx.scene.Group;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.MassType;
import virtual_robot.config.UltimateGoal;
import virtual_robot.controller.*;

import java.util.ArrayList;
import java.util.List;

@GameElementConfig(name = "Ring", filename = "ring", forGame = UltimateGoal.class, numInstances = 10)
public class Ring extends VirtualGameElement {

    public static final double RING_RADIUS_INCHES = 2.5;
    public static final List<Ring> rings = new ArrayList<>();
    public static final List<Ring> ringsOffField = new ArrayList<>();

    public enum RingStatus {
        NORMAL(true, 100, RING_FILTER),
        ROLLING(true, 0.2, RING_FILTER),
        FLYING(true, 0, RING_FLYING_FILTER),
        STACKED(true, 100, RING_STACKED_FILTER),
        CONTROLLED(false, 0, RING_FILTER),
        OFF_FIELD(false, 0, RING_FILTER);

        private boolean inPlay;
        private double linearDamping;
        private CategoryFilter filter;

        RingStatus(boolean inPlay, double linearDamping, CategoryFilter filter){
            this.inPlay = inPlay;
            this.linearDamping = linearDamping;
            this.filter = filter;
        }

        public boolean isInPlay() { return inPlay; }
        public double getLinearDamping() { return linearDamping; }
        public CategoryFilter getFilter() { return filter; }
    }


    private RingStatus status = RingStatus.OFF_FIELD;
    private RingStatus nextStatus = RingStatus.OFF_FIELD;

    private double RING_BOUNDARY;   // TODO: make this final and static (and figure out how to initialize)

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
        RING_BOUNDARY = FIELD.HALF_FIELD_WIDTH + 4 * RING_RADIUS_INCHES * FIELD.PIXELS_PER_INCH;
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

        if (Math.abs(x) > RING_BOUNDARY || Math.abs(y) > RING_BOUNDARY){
            nextStatus = RingStatus.OFF_FIELD;
        }

        setStatus(nextStatus);
    }

    @Override
    public synchronized void updateDisplay() {
        displayGroup.setVisible(status.isInPlay());
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


    public void setVelocityInchesPerSec(double vx, double vy) {
        this.ringBody.setLinearVelocity(vx / VirtualField.INCHES_PER_METER, vy / VirtualField.INCHES_PER_METER);
    }

    public void setVelocityMetersPerSec(double vx, double vy){
        this.ringBody.setLinearVelocity(vx, vy);
    }

    public RingStatus getNextStatus(){ return nextStatus; }

    public void setNextStatus(RingStatus nextStatus) { this.nextStatus = nextStatus; }

    public RingStatus getStatus() { return status; }

    public void setStatus(RingStatus status){
        this.status = status;
        this.nextStatus = status;
        setInPlay(status.isInPlay());
        if (status == RingStatus.OFF_FIELD && !ringsOffField.contains(this)) {
            ringsOffField.add(this);
        } else if (status != RingStatus.OFF_FIELD && ringsOffField.contains(this)){
            ringsOffField.remove(this);
        }
        ringBody.setLinearDamping(status.getLinearDamping());
        ringFixture.setFilter(status.getFilter());
    }

    public void setInPlay(boolean inPlay) {
        if (inPlay && !world.containsBody(elementBody)) world.addBody(elementBody);
        else if (!inPlay && world.containsBody(elementBody)) world.removeBody(elementBody);
        if (inPlay) addToDisplay();
        else removeFromDisplay();
    }

}
