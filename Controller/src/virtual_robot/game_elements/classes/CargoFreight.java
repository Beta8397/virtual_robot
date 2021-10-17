package virtual_robot.game_elements.classes;

import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.games.FreightFrenzy;

import java.util.ArrayList;
import java.util.List;

/**
 * The Cargo game elements. Behavior is derived from the Freight superclass.
 */
@GameElementConfig(name = "Cargo", filename = "cargo_freight", forGame = FreightFrenzy.class, numInstances = 20)
public class CargoFreight extends Freight {

    public static final List<CargoFreight> cargos = new ArrayList<>();  // All CargoFreight objects


}
