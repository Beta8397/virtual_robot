package virtual_robot.games;

import virtual_robot.controller.Game;

/**
 * Implementation of Game to be used when no game elements are to be used, and
 * the dyn4j physics engine is not going to be used.
 */
public final class NoGame extends Game {

    @Override
    public final void initialize() { }

    @Override
    public final void resetGameElements() { }

    @Override
    public boolean hasHumanPlayer() { return false; }

    @Override
    public final boolean isHumanPlayerAuto() { return false; }

    @Override
    public final void setHumanPlayerAuto(boolean selected) { }

    @Override
    public final void updateHumanPlayerState(double millis) { }

    @Override
    public final void requestHumanPlayerAction() { }

    @Override
    public final boolean isHumanPlayerActionRequested() { return false; }

    @Override
    public final void stopGameElements() { }
}
