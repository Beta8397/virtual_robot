package virtual_robot.games;

import java.util.*;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.NarrowphaseCollisionData;
import org.dyn4j.world.listener.CollisionListenerAdapter;
import virtual_robot.controller.Game;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.game_elements.classes.Ring;
import virtual_robot.game_elements.classes.WobbleGoal;

public class UltimateGoal extends Game {

    // Positions (inches) for wobble goals, starter stack, and ring return
    public static final Vector2[] WOBBLE_GOAL_STARTS = new Vector2[]{
            new Vector2(49.5, -48),
            new Vector2(25.5, -48)
    };
    public static final Vector2 STARTER_STACK = new Vector2(36, -22.5);
    public static final Vector2 RING_RETURN = new Vector2(24, 67);

    public static final double RING_RETURN_VELOCITY = 20.0; // inches per second
    public static final double RING_RETURN_VELOCITY_VARIATION = 2.4; // inches per second

    public static final long RING_RELEASE_INTERVAL_MILLIS = 1500;
    public static final long RING_RELEASE_INTERVAL_MILLIS_VARIATION = 500;
    private boolean humanPlayerActive = true;
    private long nextRingReleaseTimeMillis = 0L;
    private int starterStackSize = -1;
    private final Random random = new Random();


    @Override
    public void initialize() {
        //Calling the super method causes the game elements to be created, populating the gameElements list.
        super.initialize();

        //Assign the game elements to the appropriate static final lists.
        for (VirtualGameElement e: gameElements){
            if (e instanceof Ring){
                Ring.rings.add((Ring)e);
            } else if (e instanceof WobbleGoal){
                WobbleGoal.wobbles.add((WobbleGoal)e);
            }
        }

        /*
         * Add a collision listener to implement "special" handling of certain types of collision. For example,
         * this includes collisions involving rings that are stacked (to cause the stack to scatter). It should not
         * include collisions resulting in the robot controlling a game element: that should be handled within
         * the specific VirtualBot implementations.
         */
        world.addCollisionListener(new CollisionListenerAdapter<Body, BodyFixture>(){
            @Override
            public boolean collision(NarrowphaseCollisionData<Body, BodyFixture> collision) {
                return handleNarrowPhaseCollision(collision);
            }
        });

    }

    /**
     * Randomize starter stack size. Place wobbles and stacked rings on the field, and change
     * status of the stacked rings to STACKED (and all others OFF_FIELD).
     */
    @Override
    public void resetGameElements(){
        int stackIndex = new Random().nextInt(3);
        starterStackSize = new int[]{0, 1, 4}[stackIndex];

        Ring.ringsOffField.clear();

        /*
         * Place all rings off-field, then put the appropriate number of rings back on the
         * field, in the ring stack.
         */

        for (Ring r: Ring.rings){
            r.setStatus(Ring.RingStatus.OFF_FIELD);
        }

        for (int i=0; i<starterStackSize; i++){
            Ring r = Ring.rings.get(i);
            r.setStatus(Ring.RingStatus.STACKED);
            r.setLocationInches(STARTER_STACK);
        }

        /*
         * Place both wobble goals on the field, at the appropriate locations.
         */
        for (int i=0; i<2; i++){
            WobbleGoal w = WobbleGoal.wobbles.get(i);
            w.setOnField(true);
            w.setLocationInches(WOBBLE_GOAL_STARTS[i]);
        }

        updateDisplay();
    }

    @Override
    public boolean hasHumanPlayer() {
        return true;
    }

    @Override
    public boolean isHumanPlayerAuto() {
        return humanPlayerActive;
    }

    @Override
    public void setHumanPlayerAuto(boolean humanPlayerActive) {
        this.humanPlayerActive = humanPlayerActive;
    }

    /**
     * If the ring release interval has passed, and there are available off-field rings, deposit
     * a rolling ring at the release zone, with a randomized velocity.
     * @param millis milliseconds since the previous update
     */
    @Override
    public void updateHumanPlayerState(double millis) {

        if (Ring.ringsOffField.size() > 0 &&
                (System.currentTimeMillis() >= nextRingReleaseTimeMillis || humanPlayerActionRequested)) {
            Ring r = Ring.ringsOffField.get(0);
            r.setLocationInches(RING_RETURN);
            r.setStatus(Ring.RingStatus.ROLLING);
            Random random = new Random();
            double angle = (-45 - 90 * random.nextDouble()) * Math.PI / 180.0;
            double velocity = RING_RETURN_VELOCITY + RING_RETURN_VELOCITY_VARIATION * (0.5 - random.nextDouble());
            double vx = velocity * Math.cos(angle);
            double vy = velocity * Math.sin(angle);
            r.setVelocityInchesPerSec(vx, vy);
            nextRingReleaseTimeMillis = System.currentTimeMillis() + RING_RELEASE_INTERVAL_MILLIS +
                    (long) (RING_RELEASE_INTERVAL_MILLIS_VARIATION * (0.5 - random.nextDouble()));
        } else if (Ring.ringsOffField.size() == 0){
            nextRingReleaseTimeMillis = System.currentTimeMillis() + RING_RELEASE_INTERVAL_MILLIS;
        }
        humanPlayerActionRequested = false;
    }


    /**
     * Narrowphase Collision event handler
     *
     * This will be called for all collisions, but needn't do any special processing for most of them. It
     * will handle:
     *  1) collision of any body with stacked rings--give stacked ring a nudge and cancel collision.
     *  2) collision of any body with a rolling ring--set ring to not rolling and continue with collision.
     *
     *  Return true to continue processing the collision, false to stop it.
     *
     *  Note: handling of collisions that result in the robot taking control of a game element should
     *  be handled by a listener set in the VirtualBot implementation.
     */
    private boolean handleNarrowPhaseCollision(NarrowphaseCollisionData<Body, BodyFixture> collision){

        boolean result = true;

        Body b1 = collision.getBody1();
        Body b2 = collision.getBody2();
        Object o1 = b1.getUserData();
        Object o2 = b2.getUserData();

        if (o1 instanceof Ring){
            Ring r1 = (Ring)o1;
            if (r1.getStatus() == Ring.RingStatus.STACKED) {
                r1.setNextStatus(Ring.RingStatus.NORMAL);
                Vector2 v2 = b2.getLinearVelocity().copy();
                v2.rotate(Math.PI / 2 * random.nextDouble() - Math.PI / 4);
                b1.setAtRest(false);
                b1.setLinearVelocity(v2);
                result = false;
            } else if (r1.getStatus() == Ring.RingStatus.ROLLING){
                r1.setNextStatus(Ring.RingStatus.NORMAL);
            }
        }

        if (o2 instanceof Ring){
            Ring r2 = (Ring)o2;
            if (r2.getStatus() == Ring.RingStatus.STACKED) {
                r2.setNextStatus(Ring.RingStatus.NORMAL);
                Vector2 v1 = b1.getLinearVelocity().copy();
                v1.rotate(Math.PI / 2 * random.nextDouble() - Math.PI / 4);
                b2.setAtRest(false);
                b2.setLinearVelocity(v1);
                result = false;
            } else if (r2.getStatus() == Ring.RingStatus.ROLLING){
                r2.setNextStatus(Ring.RingStatus.NORMAL);
            }
        }

        return result;
    }

}
