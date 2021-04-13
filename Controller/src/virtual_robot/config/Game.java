package virtual_robot.config;

import java.util.List;

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

    /**
     * Report whether this game has a human player.  The game implementation should simulate
     * the human player's actions when the player is active.
     * @return true if game includes a human player
     */
    boolean hasHumanPlayer();

    /**
     * Report if the human player is currently active.
     * @return true if the human player is active, false otherwise
     */
    boolean isHumanPlayerActive();

    /**
     * Activate or deactivate the human player.
     * @param selected true if human player is active; false indicates inactive
     */
    void setHumanPlayerActive(boolean selected);

    /**
     * Update the state of the human player.  This is called by the VirtualRobotController during
     * the simulation loop to allow the games human player to interact with game elements.
     * @param millis milliseconds since the previous update
     */
    void updateHumanPlayerState(double millis, List<VirtualGameElement> gameElements);
}
