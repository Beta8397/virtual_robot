package virtual_robot.game_elements.classes;

import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.games.FreightFrenzy;
import virtual_robot.games.UltimateGoal;

import java.util.ArrayList;
import java.util.List;

//@GameElementConfig(name = "Box", filename = "box", forGame = FreightFrenzy.class, numInstances = 10)
public class Box extends VirtualGameElement {

    public static List<Box> boxes = new ArrayList<>();

    @Override
    public void updateState(double millis) {

    }

    @Override
    public void setUpBody() {

    }

    @Override
    public void stop() {

    }
}
