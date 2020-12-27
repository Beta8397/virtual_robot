package virtual_robot.config;

import virtual_robot.controller.VirtualGameElement;

public interface Game {
    /**
     * Initialize the game.  Implementations can choose to do one-time game initialization,
     * such as picking a starting configuration of game elements here.
     */
    void initialize();

    /**
     * Initialize a game element.  Implementations should cast the element to an appropriate
     * subclass and call an element specific initialization routine.
     * @param e the element to initialize
     * @param i the instance index of the element
     */
    void initGameElement(VirtualGameElement e, int i);
}
