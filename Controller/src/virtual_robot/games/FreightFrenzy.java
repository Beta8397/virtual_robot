package virtual_robot.games;

import javafx.scene.paint.Color;
import javafx.scene.paint.CycleMethod;
import javafx.scene.paint.LinearGradient;
import javafx.scene.paint.Stop;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.NarrowphaseCollisionData;
import org.dyn4j.world.listener.CollisionListenerAdapter;
import virtual_robot.controller.Game;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.game_elements.classes.*;

public class FreightFrenzy extends Game {

    public static final Vector2[] HUB_POSITIONS_INCHES = new Vector2[]{
            new Vector2(24, -12),       // Red hub
            new Vector2(-24, -12),      // Blue hub
            new Vector2(0, 48)          // Neutral hub
    };

    public static final Vector2 RED_CAROUSEL_POSITION_INCHES = new Vector2(68.75, -68.75);
    public static final Vector2 BLUE_CAROUSEL_POSITION_INCHES = new Vector2(-68.75, -68.75);

    private Carousel redCarousel = null;
    private Carousel blueCarousel = null;

    private boolean humanPlayerActive = false;

    @Override
    public void initialize() {
        //Calling the super method causes the game elements to be created, populating the gameElements list.
        super.initialize();

        //Assign the game elements to the appropriate static final lists.
        for (VirtualGameElement e: gameElements){
            if (e instanceof CargoFreight){
                CargoFreight.cargos.add((CargoFreight)e);
                Freight.FREIGHT_ITEMS.add((Freight)e);
            } else if (e instanceof BoxFreight){
                BoxFreight.boxes.add((BoxFreight)e);
                Freight.FREIGHT_ITEMS.add((Freight)e);
            } else if (e instanceof DuckFreight){
                DuckFreight.ducks.add((DuckFreight)e);
                Freight.FREIGHT_ITEMS.add((Freight)e);
            } else if (e instanceof ShippingHub){
                ShippingHub.shippingHubs.add((ShippingHub) e);
            } else if (e instanceof Barrier){
                Barrier.theBarrier = (Barrier) e;
            } else if (e instanceof Carousel) {
                Carousel.carousels.add((Carousel)e);
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

        redCarousel = Carousel.carousels.get(0);
        blueCarousel = Carousel.carousels.get(1);
        redCarousel.setLocationInches(RED_CAROUSEL_POSITION_INCHES);
        redCarousel.setOnField(true);
        blueCarousel.setLocationInches(BLUE_CAROUSEL_POSITION_INCHES);
        blueCarousel.setOnField(true);

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
     * This can be used for game element--game element collisions, as needed.
     * Robot--game element collisions should be handled in the specific robot config class, not here.
     *
     * For the current game implementation, no special handling of game element--game element collisions is needed;
     * the default handling by the physics engine suffices.
     *
     * However, this method can be useful for logging purposes during debugging.
     */
    private boolean handleNarrowPhaseCollision(NarrowphaseCollisionData<Body, BodyFixture> collision){

        return true;
    }

    /*
     * Set/Reset all game elements to their initial positions
     */
    @Override
    public void resetGameElements() {

        for (int i=0; i<3; i++){
            ShippingHub.shippingHubs.get(i).setOnField(true);
            ShippingHub.shippingHubs.get(i).setLocationInches(HUB_POSITIONS_INCHES[i]);
        }

        Barrier.theBarrier.setOnField(true);
        Barrier.theBarrier.setLocationInches(0, 24);

        //Before repositioning freight items, detach from shipping hubs
        for (Freight f: Freight.FREIGHT_ITEMS){
            f.setOwningShippingHub(null);
        }

        for (int i=0; i<30; i++){
            int row = i / 5;
            int col = i % 5;
            float x = row<3? -66 + col * 8 : 34 + col * 8;
            float y = 32 + (row % 3) * 16;
            BoxFreight.boxes.get(i).setOnField(true);
            BoxFreight.boxes.get(i).setLocationInches(x, y);
        }

        for (int i=0; i<20; i++){
            int row = i / 5;
            int col = i % 5;
            float x = row<2? -66 + col * 8 : 34 + col * 8;
            float y = 40 + (row % 2) * 16;
            CargoFreight.cargos.get(i).setOnField(true);
            CargoFreight.cargos.get(i).setLocationInches(x, y);
        }

        // Remove attached ducks from carousels
        redCarousel.clearAttachedDuck();
        blueCarousel.clearAttachedDuck();

        // Clear, then repopulate the red and blue loading docks, each with one half of the ducks
        int numDucks = DuckFreight.ducks.size();
        DuckFreight.redLoadingDock.clear();
        DuckFreight.blueLoadingDock.clear();
        for (int i=0; i<numDucks; i++){
            DuckFreight.ducks.get(i).setOnField(false);
            if (i< numDucks/2) DuckFreight.redLoadingDock.add(DuckFreight.ducks.get(i));
            else DuckFreight.blueLoadingDock.add(DuckFreight.ducks.get(i));
        }

        // Attach a duck to each carousel, and remove from the corresponding loading dock
        redCarousel.attachDuck(DuckFreight.redLoadingDock.get(0));
        DuckFreight.redLoadingDock.remove(0);
        blueCarousel.attachDuck(DuckFreight.blueLoadingDock.get(0));
        DuckFreight.blueLoadingDock.remove(0);

        // Simulation isn't running when this method is called, so explicit display update required
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

    /**
     * Update the human player state. If humanPlaterActive is true, this method will be called once during
     * each iteration of the game loop. Otherwise, it will be called during the game loop only if
     * humanPlayerActionRequested is true (and that is set using the Human Action button).
     * @param millis milliseconds since the previous update
     */
    @Override
    public void updateHumanPlayerState(double millis) {
        /*
         * For each carousel, attach a duck only if: there is currently no attached duck AND at least 1 second
         * has passed since prior duck release AND there is at least one duck remaining in the corresponding loading
         * dock AND the carousel has stopped spinning.
         */
        if (redCarousel.getAttachedDuck() == null
                && redCarousel.getTimerMilliseconds() > 1000 && DuckFreight.redLoadingDock.size() > 0
                && Math.abs(redCarousel.getElementBody().getAngularVelocity()) < 0.01) {
            redCarousel.attachDuck(DuckFreight.redLoadingDock.get(0));
            DuckFreight.redLoadingDock.remove(0);
        }
        if (blueCarousel.getAttachedDuck() == null
                && blueCarousel.getTimerMilliseconds() > 1000 && DuckFreight.blueLoadingDock.size() > 0
                && Math.abs(blueCarousel.getElementBody().getAngularVelocity()) < 0.01) {
            blueCarousel.attachDuck(DuckFreight.blueLoadingDock.get(0));
            DuckFreight.blueLoadingDock.remove(0);
        }
        humanPlayerActionRequested = false;
    }

}
