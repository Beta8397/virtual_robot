package virtual_robot.game_elements.classes;

import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.MassType;
import virtual_robot.controller.Filters;
import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;
import virtual_robot.games.CenterStage;
import virtual_robot.games.FreightFrenzy;

import java.util.ArrayList;
import java.util.List;

@GameElementConfig(name = "Backdrop", filename = "backdrop", forGame = CenterStage.class, numInstances = 2)
public class Backdrop extends VirtualGameElement {

    public static final List<Backdrop> backdrops = new ArrayList<>();

    Body backdropBody = null;

    // Category and filter for collisions
    public static final long BACKDROP_CATEGORY = 1024;
    public static final CategoryFilter BACKDROP_FILTER = new CategoryFilter(BACKDROP_CATEGORY, Filters.MASK_ALL & ~Filters.ARM);

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
        x = backdropBody.getTransform().getTranslationX() * VirtualField.PIXELS_PER_METER;
        y = backdropBody.getTransform().getTranslationY() * VirtualField.PIXELS_PER_METER;
        headingRadians = backdropBody.getTransform().getRotationAngle();
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
        elementBody = Dyn4jUtil.createBody(displayGroup, this, 0, 0, new FixtureData(BACKDROP_FILTER, 1, 0, 0));
        backdropBody = elementBody;       // Alias for elementBody
        backdropBody.setMass(MassType.INFINITE);
    }
}
