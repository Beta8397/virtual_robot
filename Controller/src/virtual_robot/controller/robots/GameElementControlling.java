package virtual_robot.controller.robots;

import java.util.List;

import virtual_robot.controller.VirtualGameElement;

public interface GameElementControlling {
    /**
     * If a robot implements this interface, this method will be called by the VirtualRobotController
     * when initializing the robot to allow the robot to select game elements to start controlling,
     * e.g., preloaded game elements.
     * @param gameElements a list of all non-robot game elements
     */
    void initControl(List<VirtualGameElement> gameElements);

    /**
     * Implement this method to provide interactions between a bot and game elements.  The
     * VirtualRobotController will call this method once for each game element each time through
     * the simulation loop.
     * @param e the game element
     */
    void interact(VirtualGameElement e);
}
