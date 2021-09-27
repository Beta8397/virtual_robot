package virtual_robot.game_elements.classes;

import virtual_robot.controller.VirtualGameElement;

import java.util.ArrayList;
import java.util.List;

//@GameElementConfig(name = "Box", filename = "box_freight", forGame = FreightFrenzy.class, numInstances = 10)
public class BoxFreight extends Freight {

    public static List<BoxFreight> boxes = new ArrayList<>();

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
