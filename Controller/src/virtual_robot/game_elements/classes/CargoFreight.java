package virtual_robot.game_elements.classes;

import virtual_robot.controller.VirtualGameElement;

import java.util.ArrayList;
import java.util.List;

//@GameElementConfig(name = "Cargo", filename = "cargo", forGame = FreightFrenzy.class, numInstances = 10)
public class CargoFreight extends Freight {

    public static final List<CargoFreight> cargos = new ArrayList<>();

    @Override
    public void updateState(double millis) {
        //TODO: Code to update state of cargo item
    }

    @Override
    public void setUpBody() {
        //TODO: Set up a dyn4j Body for the cargo elements and assign it to elementBody
    }

    @Override
    public void stop() {
        //
    }
}
