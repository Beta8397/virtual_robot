package virtual_robot.games;

import javafx.scene.paint.Color;
import javafx.scene.paint.CycleMethod;
import javafx.scene.paint.LinearGradient;
import javafx.scene.paint.Stop;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.world.NarrowphaseCollisionData;
import org.dyn4j.world.listener.CollisionListenerAdapter;
import virtual_robot.controller.Game;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.game_elements.classes.*;

public class FreightFrenzy extends Game {

    private boolean humanPlayerActive = false;

    @Override
    public void initialize() {
        //Calling the super method causes the game elements to be created, populating the gameElements list.
        super.initialize();

        //Assign the game elements to the appropriate static final lists.
        for (VirtualGameElement e: gameElements){
            if (e instanceof Cargo){
                Cargo.cargos.add((Cargo)e);
            } else if (e instanceof Box){
                Box.boxes.add((Box)e);
            } else if (e instanceof Duck){
                Duck.ducks.add((Duck)e);
            } else if (e instanceof ShippingHub){
                ShippingHub.shippingHubs.add((ShippingHub) e);
            } else if (e instanceof Barrier){
                Barrier.theBarrier = (Barrier) e;
            }
        }

        /*
         * Set the Shipping Hub Colors -- Red, Blue, and Green (for neutral)
         */
        ShippingHub.shippingHubs.get(0).getOuterCircle().setFill(Color.valueOf("FF0000"));
        ShippingHub.shippingHubs.get(1).getOuterCircle().setFill(Color.valueOf("0000FF"));

        Stop blueStop = new Stop(0.49, Color.BLUE);
        Stop redStop = new Stop(0.51, Color.RED);
        LinearGradient gradient = new LinearGradient(0, 0, 1, 0, true,
                CycleMethod.NO_CYCLE, blueStop, redStop);
        ShippingHub.shippingHubs.get(2).getOuterCircle().setFill(gradient);


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
     * Narrowphase Collision event handler
     *
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

     //TODO: ADD COLLISION-HANDLING CODE HERE

        return true;
    }

    @Override
    public void resetGameElements() {
        //TODO: ADD CODE TO RESET GAME ELEMENTS

        for (ShippingHub sh: ShippingHub.shippingHubs) sh.setOnField(true);
        ShippingHub.shippingHubs.get(0).setLocationInches(24, -12);
        ShippingHub.shippingHubs.get(1).setLocationInches(-24, -12);
        ShippingHub.shippingHubs.get(2).setLocationInches(0, 48);

        Barrier.theBarrier.setOnField(true);
        Barrier.theBarrier.setLocationInches(0, 24);

        updateDisplay();
    }

    /**
     * Freight Frenzy effectively does use a human player, so return true.
     * @return
     */
    @Override
    public boolean hasHumanPlayer() {
        return true;
    }

    @Override
    public boolean isHumanPlayerAuto() {
        return humanPlayerActive;
    }

    @Override
    public void setHumanPlayerAuto(boolean selected) {
        humanPlayerActive = selected;
    }

    @Override
    public void updateHumanPlayerState(double millis) {
        //TODO: ADD CODE TO UPDATE HUMAN PLAYER STATE (If no duck on carousel and duck available, add duck to carousel)
    }

}
