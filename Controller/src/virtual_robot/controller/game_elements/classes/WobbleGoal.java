package virtual_robot.controller.game_elements.classes;

import javafx.scene.Group;
import virtual_robot.config.UltimateGoal;
import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.util.Vector2D;

@GameElementConfig(name = "Wobble Goal", filename = "wobble_goal", forGame = UltimateGoal.class, numInstances = 2)
public class WobbleGoal extends VirtualGameElement {
    public static final double RADIUS_INCHES = 4.0;
    private VirtualBot bot;

    @Override
    protected void setUpDisplayGroup(Group group) {
        super.setUpDisplayGroup(group);
    }

    @Override
    public void updateState(double millis) {
        // TODO make wobble goal wobble?
    }

    @Override
    public synchronized void updateDisplay() {
        super.updateDisplay();
    }

    @Override
    public VirtualBot getControlledBy() {
        return bot;
    }

    @Override
    public void setControlledBy(VirtualBot bot) {
        this.bot = bot;
    }
}
