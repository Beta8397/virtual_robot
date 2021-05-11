package virtual_robot.controller;

import java.lang.annotation.Annotation;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import javafx.fxml.FXMLLoader;
import javafx.scene.Group;
import javafx.scene.layout.Pane;
import org.dyn4j.dynamics.Body;
import org.dyn4j.world.World;
import org.reflections.Reflections;
import virtual_robot.config.Config;

public abstract class Game {

    protected static VirtualRobotController controller;

    protected Pane fieldPane = null;

    // The dyn4j physics world.
    protected World<Body> world = null;

    // List to hold all of the game elements for the Game instance.
    protected List<VirtualGameElement> gameElements = new ArrayList<VirtualGameElement>();

    // Flag to indicate whether human player action has been requested.
    protected volatile boolean humanPlayerActionRequested;

    public Game(){ }

    /**
     * Set a reference to the VirtualRobotController. This must be called before the
     * initialize() method can be called on an instance of Game.
     * @param c
     */
    public static void setController(VirtualRobotController c){
        controller = c;
    }

    /**
     * Initialize the game.
     * Implementation may override, but must include super.initialize() as first statement.
     * Game.setController must be called to obtain the controller reference before the
     * initialize method can be called.
     */
    public void initialize(){
        world = controller.getWorld();
        fieldPane = controller.getFieldPane();
        createGameElements();
    }

    /**
     *  Create all game elements and add them to gameElements list.
     */
    private void createGameElements() {
        // iterate through annotated game element classes; filter for classes having forGame == Config.GAME
        Reflections reflections = new Reflections("virtual_robot.game_elements.classes");
        Set<Class<?>> configClasses = new HashSet<>();
        configClasses.addAll(reflections.getTypesAnnotatedWith(GameElementConfig.class));
        for (Class<?> c : configClasses) {
            Class<? extends Game> forGame = c.getAnnotation(GameElementConfig.class).forGame();
            int numInstances = c.getAnnotation(GameElementConfig.class).numInstances();
            if (forGame.equals(Config.GAME.getClass()) && numInstances > 0 && VirtualGameElement.class.isAssignableFrom(c)) {
                // create numInstances instances of each
                for (int i = 0; i < numInstances; ++i) {
                    gameElements.add(getVirtualGameElementInstance(c));
                }
            }
        }
    }

    /**
     * Set all game elements to their starting positions, whether on or off the field.
     */
    public abstract void resetGameElements();

    /**
     * Create and return single instance of game element specified by the class c.
     * @param c
     * @return
     */
    private VirtualGameElement getVirtualGameElementInstance(Class<?> c){
        try {
            Annotation a = c.getAnnotation(GameElementConfig.class);
            FXMLLoader loader = new FXMLLoader((getClass().getResource("/virtual_robot/game_elements/fxml/" + ((GameElementConfig) a).filename() + ".fxml")));
            Group group = (Group) loader.load();
            VirtualGameElement element = (VirtualGameElement) loader.getController();
            element.setUpGameElement(group);
            return element;
        } catch (Exception e){
            System.out.println("Unable to load game element configuration.");
            System.out.println(e.getMessage());
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Report whether this game has a human player.  The game implementation should simulate
     * the human player's actions when the player is active.
     * @return true if game includes a human player
     */
    public abstract boolean hasHumanPlayer();

    /**
     * Report if the human player is currently active.
     * @return true if the human player is active, false otherwise
     */
    public abstract boolean isHumanPlayerAuto();

    /**
     * Activate or deactivate the human player.
     * @param selected true if human player is active; false indicates inactive
     */
    public abstract void setHumanPlayerAuto(boolean selected);

    /**
     * Update the state of the human player.  This is called by the VirtualRobotController during
     * the simulation loop to allow the games human player to interact with game elements.
     * @param millis milliseconds since the previous update
     */
    public abstract void updateHumanPlayerState(double millis);

    public void requestHumanPlayerAction(){
        humanPlayerActionRequested = true;
    }

    public boolean isHumanPlayerActionRequested() { return humanPlayerActionRequested; }

    /**
     * Update state of all game elements.
     * @param millis
     */
    public void updateGameElementState(double millis){
        for (VirtualGameElement e: gameElements){
            e.updateState(millis);
        }
    }

    /**
     *  Update display of all game elements.
     */
    public void updateDisplay(){
        for (VirtualGameElement e: gameElements){
            e.updateDisplay();
        }
    }

    public synchronized void stopGameElements(){
        for (VirtualGameElement e: gameElements) e.stop();
    }

}
