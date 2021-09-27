package virtual_robot.game_elements.classes;

import virtual_robot.controller.VirtualGameElement;

import java.util.ArrayList;
import java.util.List;

//@GameElementConfig(name = "Cargo", filename = "cargo", forGame = FreightFrenzy.class, numInstances = 10)
public class DuckFreight extends Freight {

    public static List<DuckFreight> ducks = new ArrayList<>();

    @Override
    public void updateState(double millis) {

    }

    @Override
    public void setUpBody() {

    }

}
