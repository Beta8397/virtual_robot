package virtual_robot.game_elements.classes;

import javafx.scene.Group;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.MassType;
import virtual_robot.controller.Filters;
import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;
import virtual_robot.games.FreightFrenzy;

@GameElementConfig(name = "Barrier", filename = "barrier", forGame = FreightFrenzy.class, numInstances = 1)
public class Barrier extends VirtualGameElement {

    public static Barrier theBarrier = null;

    Body barrierBody = null;

    // Category and filter for collisions
    public static final long BARRIER_CATEGORY = 1024;
    public static final CategoryFilter BARRIER_FILTER = new CategoryFilter(BARRIER_CATEGORY, Filters.MASK_ALL & ~Filters.CHASSIS & ~Filters.ARM);

    public void initialize(){
        super.initialize();
    }


    /**
     * The implementation of updateState for Barrier is simple: just obtain (x, y, headingRadians)
     * from the physics body, and translate x and y from meters to pixels.
     * @param millis milliseconds since the previous update
     */
    @Override
    public void updateState(double millis) {
        x = barrierBody.getTransform().getTranslationX() * VirtualField.PIXELS_PER_METER;
        y = barrierBody.getTransform().getTranslationY() * VirtualField.PIXELS_PER_METER;
        headingRadians = barrierBody.getTransform().getRotationAngle();
    }

    @Override
    public synchronized void updateDisplay() {
        super.updateDisplay();
    }

    /**
     * Set up the dyn4j physics body for the barrier.
     */
    @Override
    public void setUpBody(){
        /*
         * Use Dyn4jUtil.createBody to create a Body. Infinite mass (i.e., the barrier is fixed in position)
         */
        elementBody = Dyn4jUtil.createBody(displayGroup, this, 0, 0, new FixtureData(BARRIER_FILTER, 1, 0, 0));
        barrierBody = elementBody;       // Alias for elementBody
        barrierBody.setMass(MassType.INFINITE);
    }
}
