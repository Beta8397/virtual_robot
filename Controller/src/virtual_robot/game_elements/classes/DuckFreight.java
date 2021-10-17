package virtual_robot.game_elements.classes;

import org.dyn4j.collision.CategoryFilter;
import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.games.FreightFrenzy;

import java.util.ArrayList;
import java.util.List;

/**
 * The Duck game elements. Behavior is derived from the Freight parent class.
 */
@GameElementConfig(name = "Duck", filename = "duck_freight", forGame = FreightFrenzy.class, numInstances = 20)
public class DuckFreight extends Freight {

    public static List<DuckFreight> ducks = new ArrayList<>();              // All DuckFreight objects
    public static List<DuckFreight> redLoadingDock = new ArrayList<>();     // DuckFreight objects in red loading dock
    public static List<DuckFreight> blueLoadingDock = new ArrayList<>();    // DuckFreight objects in blue loading dock

}
