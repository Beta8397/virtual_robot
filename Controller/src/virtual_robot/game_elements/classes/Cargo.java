package virtual_robot.game_elements.classes;

import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.games.FreightFrenzy;
import virtual_robot.games.UltimateGoal;

import java.util.ArrayList;
import java.util.List;

//@GameElementConfig(name = "Cargo", filename = "cargo", forGame = FreightFrenzy.class, numInstances = 10)
public class Cargo extends VirtualGameElement {

    public static final List<Cargo> cargos = new ArrayList<>();

    @Override
    public void updateState(double millis) {
        //TODO: Code to update state of cargo item
    }

    @Override
    public void setUpBody() {
        //TODO: Set up a dyn4j Body for the cargo elements and assign it to elementBody
    }

}
