package virtual_robot.config;

import java.util.*;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.NarrowphaseCollisionData;
import org.dyn4j.world.listener.CollisionListenerAdapter;
import virtual_robot.controller.VRBody;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.controller.game_elements.classes.Ring;
import virtual_robot.controller.game_elements.classes.WobbleGoal;
import virtual_robot.util.Vector2D;

public class UltimateGoal extends Game {

    private List<Ring> rings = new ArrayList<>();
    private List<Ring> ringsOffField = new ArrayList<>();
    private List<WobbleGoal> wobbles = new ArrayList<>();

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
        super.initialize();

        for (VirtualGameElement e: gameElements){
            if (e instanceof Ring){
                rings.add((Ring)e);
            } else if (e instanceof WobbleGoal){
                wobbles.add((WobbleGoal)e);
            }
        }

        world.addCollisionListener(new CollisionListenerAdapter<VRBody, BodyFixture>(){
            @Override
            public boolean collision(NarrowphaseCollisionData<VRBody, BodyFixture> collision) {
                return handleNarrowPhaseCollision(collision);
            }
        });

    }

    @Override
    public void resetGameElements(){
        int stackIndex = new Random().nextInt(3);
//        starterStackSize = new int[]{0, 1, 4}[stackIndex];
        starterStackSize = 4;

        ringsOffField.clear();

        for (Ring r: rings){
            r.setOnField(false);
            r.setInFlight(false);
            r.setRolling(false);
            r.setControlled(false);
            ringsOffField.add(r);
        }

        System.out.println("Rings: " + rings.size() + "  Rings off field: " + ringsOffField.size());

        for (int i=0; i<starterStackSize; i++){
            Ring r = rings.get(i);
            r.setOnField(true);
            r.setStacked(true);
            r.setLocationInches(STARTER_STACK);
            ringsOffField.remove(r);
        }

        for (int i=0; i<2; i++){
            WobbleGoal w = wobbles.get(i);
            w.setOnField(true);
            w.setLocationInches(WOBBLE_GOAL_STARTS[i]);
        }

        updateDisplay();
    }

    public List<Ring> getRings(){ return rings; }

    public List<Ring> getRingsOffField() { return ringsOffField; }

    public List<WobbleGoal> getWobbles() { return wobbles; }

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

    /**
     * If the ring release interval has passed, and there are available off-field rings, deposit
     * a rolling ring at the release zone, with a randomized velocity.
     * @param millis milliseconds since the previous update
     */
    @Override
    public void updateHumanPlayerState(double millis) {

        if (ringsOffField.size() > 0 && System.currentTimeMillis() >= nextRingReleaseTimeMillis) {
            Ring r = ringsOffField.get(0);
            r.setLocationInches(RING_RETURN);
            r.setOnField(true);
            ringsOffField.remove(r);
            r.setInFlight(false);
            r.setRolling(true);
            Random random = new Random();
            double angle = (-45 - 90 * random.nextDouble()) * Math.PI / 180.0;
            double velocity = RING_RETURN_VELOCITY + RING_RETURN_VELOCITY_VARIATION * (0.5 - random.nextDouble());
            double vx = velocity * Math.cos(angle);
            double vy = velocity * Math.sin(angle);
            r.setVelocityInchesPerSec(vx, vy);
            nextRingReleaseTimeMillis = System.currentTimeMillis() + RING_RELEASE_INTERVAL_MILLIS +
                    (long) (RING_RELEASE_INTERVAL_MILLIS_VARIATION * (0.5 - random.nextDouble()));
        } else if (ringsOffField.size() == 0){
            nextRingReleaseTimeMillis = System.currentTimeMillis() + RING_RELEASE_INTERVAL_MILLIS;
        }
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
    private boolean handleNarrowPhaseCollision(NarrowphaseCollisionData<VRBody, BodyFixture> collision){

        boolean result = true;

        VRBody b1 = collision.getBody1();
        VRBody b2 = collision.getBody2();
        GameObject o1 = b1.getParent();
        GameObject o2 = b2.getParent();

        if (o1 instanceof Ring){
            Ring r1 = (Ring)o1;
            if (r1.isStacked()) {
                r1.setStacked(false);
                Vector2 v2 = b2.getLinearVelocity().copy();
                v2.rotate(Math.PI / 2 * random.nextDouble() - Math.PI / 4);
                b1.setAtRest(false);
                b1.setLinearVelocity(v2);
                result = false;
            } else if (r1.isRolling()){
                r1.setRolling(false);
            }
        }

        if (o2 instanceof Ring && ((Ring)o2).isStacked()){
            Ring r2 = (Ring)o2;
            if (r2.isStacked()) {
                r2.setStacked(false);
                Vector2 v1 = b1.getLinearVelocity().copy();
                v1.rotate(Math.PI / 2 * random.nextDouble() - Math.PI / 4);
                b2.setAtRest(false);
                b2.setLinearVelocity(v1);
                result = false;
            } else if (r2.isRolling()){
                r2.setRolling(false);
            }
        }

        return result;
    }

}
