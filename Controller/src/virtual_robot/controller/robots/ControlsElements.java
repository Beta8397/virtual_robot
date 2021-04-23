package virtual_robot.controller.robots;

import java.util.List;

import virtual_robot.config.Game;
import virtual_robot.controller.VirtualGameElement;

public interface ControlsElements {
    void preloadElements(Game game);
    void clearLoadedElements(Game game);
}
