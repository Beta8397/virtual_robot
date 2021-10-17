package virtual_robot.game_elements.classes;

import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.games.FreightFrenzy;

import java.util.ArrayList;
import java.util.List;

/**
 * Box game elements. Behavior is derived from the Freight superclass.
 */
@GameElementConfig(name = "Box", filename = "box_freight", forGame = FreightFrenzy.class, numInstances = 31)
public class BoxFreight extends Freight {

    public static List<BoxFreight> boxes = new ArrayList<>();   // List of all BoxFreight objects.

}
