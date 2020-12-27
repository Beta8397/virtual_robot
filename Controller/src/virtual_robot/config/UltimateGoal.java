package virtual_robot.config;

import java.util.Random;

import virtual_robot.controller.VirtualGameElement;
import virtual_robot.controller.game_elements.classes.Ring;
import virtual_robot.controller.game_elements.classes.WobbleGoal;
import virtual_robot.util.Vector2D;

public class UltimateGoal implements Game {
    public static final Vector2D[] WOBBLE_GOAL_STARTS = new Vector2D[]{
            new Vector2D(49.5, -48),
            new Vector2D(25.5, -48)
    };
    public static final Vector2D STARTER_STACK = new Vector2D(36, -22.5); // TODO fix this
    private int starterStackSize = -1; // requires call to initialize()
    private int ringCount = 0; // temporary variable for tracking rings added to starter stack

    @Override
    public void initialize() {
        // handle starter stack randomization
        Random random = new Random();
        starterStackSize = new int[] {0, 1, 4}[(int) Math.floor(3 * random.nextDouble())];
        ringCount = 0;
    }


    @Override
    public void initGameElement(VirtualGameElement e, int i) {
        if (e instanceof WobbleGoal && i < WOBBLE_GOAL_STARTS.length) {
            WobbleGoal w = (WobbleGoal) e;
            if (w.getControlledBy() == null) {  // ignore preloaded wobble goal
                e.setLocationInInches(WOBBLE_GOAL_STARTS[i]);
            }
        } else if (e instanceof Ring) {
            Ring r = (Ring) e;
            if (r.getControlledBy() == null) {  // ignore rings preloaded on the robot
                if (ringCount < starterStackSize) {
                    // put ring in starter stack
                    r.setLocationInInches(STARTER_STACK); // TODO handle stacking of rings
                    r.setOnField(true);
                    ++ringCount;
                } else {
                    r.setOnField(false);
                }
            }
        }
    }
}
