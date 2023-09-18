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

@GameElementConfig(name = "TrussLeg", filename = "trussleg", forGame = CenterStage.class, numInstances = 12)
public class TrussLeg extends VirtualGameElement {

    public static final List<TrussLeg> trussLegs = new ArrayList<>();

    Body trussLegBody = null;

    // Category and filter for collisions
    public static final long TRUSSLEG_CATEGORY =2048;
    public static final CategoryFilter TRUSSLEG_FILTER = new CategoryFilter(TRUSSLEG_CATEGORY, Filters.MASK_ALL);

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
        x = trussLegBody.getTransform().getTranslationX() * VirtualField.PIXELS_PER_METER;
        y = trussLegBody.getTransform().getTranslationY() * VirtualField.PIXELS_PER_METER;
        headingRadians = trussLegBody.getTransform().getRotationAngle();
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
        elementBody = Dyn4jUtil.createBody(displayGroup, this, 0, 0,
                new FixtureData(TRUSSLEG_FILTER, 1, 0, 0, 2, 1));
        trussLegBody = elementBody;       // Alias for elementBody
        trussLegBody.setMass(MassType.INFINITE);
    }
}
