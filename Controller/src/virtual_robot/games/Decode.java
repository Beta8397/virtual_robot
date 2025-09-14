package virtual_robot.games;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Polygon;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Triangle;
import org.dyn4j.geometry.Vector2;

import virtual_robot.controller.Filters;
import virtual_robot.controller.Game;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.Wall;

/**
 * Decode is just like NoGame, except that there will be stationary (infinite mass) bodies
 * representing the Goals and Ramps
 */
public final class Decode extends Game {

    @Override
    public final void initialize() {
        super.initialize();

        Polygon blueGoalPoly = new Polygon(new Vector2(0,0), new Vector2(0,-0.547),
                new Vector2(0.217, -0.547), new Vector2(0.604,0));
        Rectangle blueRampRect = new Rectangle(0.217, 1.282);
        Polygon redGoalPoly = new Polygon(new Vector2(0,0), new Vector2(-0.624,0),
                new Vector2(-0.217,-0.547), new Vector2(0, -0.547));
        Rectangle redRampRect = new Rectangle(0.217, 1.282);

        blueGoalPoly.translate(-VirtualField.HALF_FIELD_WIDTH_METERS, VirtualField.HALF_FIELD_WIDTH_METERS);
        blueRampRect.translate(-VirtualField.HALF_FIELD_WIDTH_METERS + blueRampRect.getWidth()/2.0,
                blueRampRect.getHeight()/2.0);
        redGoalPoly.translate(VirtualField.HALF_FIELD_WIDTH_METERS, VirtualField.HALF_FIELD_WIDTH_METERS);
        redRampRect.translate(VirtualField.HALF_FIELD_WIDTH_METERS-redRampRect.getWidth()/2.0,
                redRampRect.getHeight()/2.0);

        Convex[] convexes = new Convex[]{blueGoalPoly, blueRampRect, redGoalPoly, redRampRect};
        for (Convex convex: convexes){
            Body body = new Body();
            BodyFixture fixture = body.addFixture(convex);
            fixture.setFilter(Filters.WALL_FILTER);
            body.setMass(MassType.INFINITE);
            world.addBody(body);
        }
    }

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
