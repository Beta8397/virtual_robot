package virtual_robot.game_elements.classes;

import org.dyn4j.collision.CategoryFilter;
import virtual_robot.controller.Filters;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;

import java.util.ArrayList;
import java.util.List;

public abstract class Freight extends VirtualGameElement {

    public static final List<Freight> freightItems = new ArrayList<>();

    public static long FREIGHT_CATEGORY = 2048;
    public static CategoryFilter FREIGHT_FILTER = new CategoryFilter(FREIGHT_CATEGORY, Filters.MASK_ALL);

    @Override
    public void setUpBody(){
        /*
         * Use Dyn4jUtil.createBody to create a Body. Infinite mass (i.e., the barrier is fixed in position)
         */
        elementBody = Dyn4jUtil.createBody(displayGroup, this, 0, 0,
                new FixtureData(FREIGHT_FILTER, 1, 0, 0));
        elementBody.setLinearDamping(100);
    }

}
