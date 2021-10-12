package virtual_robot.game_elements.classes;

import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.games.FreightFrenzy;

import java.util.ArrayList;
import java.util.List;

@GameElementConfig(name = "Duck", filename = "duck_freight", forGame = FreightFrenzy.class, numInstances = 20)
public class DuckFreight extends Freight {

    public static List<DuckFreight> ducks = new ArrayList<>();
    public static List<DuckFreight> ducksOffFieldRed = new ArrayList<>();
    public static List<DuckFreight> ducksOffFieldBlue = new ArrayList<>();

}
