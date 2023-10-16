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
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.game_elements.classes.*;

public class CenterStage extends Game {

    public static final Vector2[] BACKDROP_POSITIONS_INCHES = new Vector2[]{
            new Vector2(-35, 0.5*VirtualField.FIELD_WIDTH_INCHES - 6.625),       // Blue Side
            new Vector2(36, 0.5*VirtualField.FIELD_WIDTH_INCHES - 6.625)      // Red Side
    };

    public static final Vector2[] TRUSS_LEG_POSITIONS_INCHES = new Vector2[]{
            new Vector2(-0.5*VirtualField.FIELD_WIDTH_INCHES + 2, -2),
            new Vector2(-0.5*VirtualField.FIELD_WIDTH_INCHES + 2, -22),
            new Vector2(-47, -2), new Vector2(-47, -22),
            new Vector2(-23.5, -2), new Vector2(-23.5, -22),
            new Vector2(24, -2), new Vector2(24, -22),
            new Vector2(48, -2), new Vector2(48, -22),
            new Vector2(0.5*VirtualField.FIELD_WIDTH_INCHES - 1.5, -2),
            new Vector2(0.5*VirtualField.FIELD_WIDTH_INCHES - 1.5, -22)
    };


    private boolean humanPlayerActive = false;

    @Override
    public void initialize() {
        //Calling the super method causes the game elements to be created, populating the gameElements list.
        super.initialize();

        //Assign the game elements to the appropriate static final lists.
        for (VirtualGameElement e: gameElements){
            if (e instanceof Backdrop){
                Backdrop.backdrops.add((Backdrop)e);
            } else if (e instanceof TrussLeg){
                TrussLeg.trussLegs.add((TrussLeg)e);
            }
        }

        Backdrop.backdrops.get(0).setLocationInches(BACKDROP_POSITIONS_INCHES[0]);
        Backdrop.backdrops.get(0).setOnField(true);
        Backdrop.backdrops.get(1).setLocationInches(BACKDROP_POSITIONS_INCHES[1]);
        Backdrop.backdrops.get(1).setOnField(true);

        for (int i=0; i<12; i++){
            TrussLeg.trussLegs.get(i).setLocationInches(TRUSS_LEG_POSITIONS_INCHES[i]);
            TrussLeg.trussLegs.get(i).setOnField(true);
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

        Backdrop.backdrops.get(0).setLocationInches(BACKDROP_POSITIONS_INCHES[0]);
        Backdrop.backdrops.get(0).setOnField(true);
        Backdrop.backdrops.get(1).setLocationInches(BACKDROP_POSITIONS_INCHES[1]);
        Backdrop.backdrops.get(1).setOnField(true);

        for (int i=0; i<12; i++){
            TrussLeg.trussLegs.get(i).setLocationInches(TRUSS_LEG_POSITIONS_INCHES[i]);
            TrussLeg.trussLegs.get(i).setOnField(true);
        }


        // Simulation isn't running when this method is called, so explicit display update required
        updateDisplay();
    }

    /**
     * Freight Frenzy effectively does use a human player, so return true.
     * @return
     */
    @Override
    public boolean hasHumanPlayer() {
        return false;
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
    }

}
