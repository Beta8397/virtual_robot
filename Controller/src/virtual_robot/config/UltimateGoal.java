package virtual_robot.config;

import java.util.ArrayList;
import java.util.List;
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
    public static final Vector2D STARTER_STACK = new Vector2D(36, -22.5);
    public static final Vector2D RING_RETURN = new Vector2D(24, 67);
    public static final double RING_RETURN_VELOCITY = 50.0; // pixels per second
    public static final double RING_RETURN_VELOCITY_VARIATION = 5.0; // pixels per second
    public static final long RING_RELEASE_INTERVAL_MILLIS = 1500;
    public static final long RING_RELEASE_INTERVAL_MILLIS_VARIATION = 500;
    private int starterStackSize = -1; // requires call to initialize()
    private int ringCount = 0; // temporary variable for tracking rings added to starter stack
    private boolean humanPlayerActive = false;
    private long nextRingReleaseTimeMillis = 0L;

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

    @Override
    public boolean hasHumanPlayer() {
        return true;
    }

    @Override
    public boolean isHumanPlayerActive() {
        return humanPlayerActive;
    }

    @Override
    public void setHumanPlayerActive(boolean humanPlayerActive) {
        this.humanPlayerActive = humanPlayerActive;
    }

    @Override
    public void updateHumanPlayerState(double millis, List<VirtualGameElement> gameElements) {
        List<Ring> ringsOffField = new ArrayList<>();
        for (VirtualGameElement e : gameElements) {
            if (e instanceof Ring) {
                Ring r = (Ring) e;
                if (!r.isOnField() && r.getControlledBy() == null) {
                    ringsOffField.add(r);
                }
            }
        }

        if (ringsOffField.size() > 0 && System.currentTimeMillis() >= nextRingReleaseTimeMillis) {
            Ring r = ringsOffField.get(0);
            r.setLocationInInches(RING_RETURN);
            r.setOnField(true);
            r.setInFlight(false);
            r.setRolling(true);
            Random random = new Random();
            double angle = (-45 - 90 * random.nextDouble()) * Math.PI / 180.0;
            double velocity = RING_RETURN_VELOCITY + RING_RETURN_VELOCITY_VARIATION * (0.5 - random.nextDouble());
            double vx = velocity * Math.cos(angle);
            double vy = velocity * Math.sin(angle);
            r.setVelocity(vx, vy);
            nextRingReleaseTimeMillis = System.currentTimeMillis() + RING_RELEASE_INTERVAL_MILLIS +
                    (long) (RING_RELEASE_INTERVAL_MILLIS_VARIATION * (0.5 - random.nextDouble()));
        }
        else if (ringsOffField.size() == 0){
            nextRingReleaseTimeMillis = System.currentTimeMillis() + RING_RELEASE_INTERVAL_MILLIS;
        }
    }
}
