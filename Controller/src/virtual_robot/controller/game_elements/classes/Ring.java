package virtual_robot.controller.game_elements.classes;

import javafx.scene.Group;
import virtual_robot.config.UltimateGoal;
import virtual_robot.controller.GameElementConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;

@GameElementConfig(name = "Ring", filename = "ring", forGame = UltimateGoal.class, numInstances = 10)
public class Ring extends VirtualGameElement {
    public static final double RING_RADIUS_INCHES = 2.5;
    public static final double DRAG = -0.1;
    private boolean onField = true;
    private boolean inFlight = false;
    private boolean rolling = false;
    private VirtualBot bot = null;
    private double vx = 0.0; // pixels per sec
    private double vy = 0.0; // pixels per sec

    @Override
    protected void setUpDisplayGroup(Group group) {
        super.setUpDisplayGroup(group);
    }

    @Override
    public synchronized void updateState(double millis) {
        x += vx * millis / 1000.0;
        y += vy * millis / 1000.0;

        VirtualField field = controller.getField();

        if (isInFlight()) {
            if (x < field.X_MIN || x > field.X_MAX || y < field.Y_MIN || y > field.Y_MAX) {
                // ring exited field
                onField = false;
                inFlight = false;
                rolling = false;
                vx = 0.0;
                vy = 0.0;
            }
        }
        else if (getVx() != 0.0 || getVy() != 0.0) {
            // when rolling on the ground, apply the acceleration
            double angle = Math.atan2(vy, vx);
            vx += vx * DRAG * Math.cos(angle) * millis / 1000.0;
            vy += vy * DRAG * Math.sin(angle) * millis / 1000.0;

            if (Math.abs(vx) < 1.0e-5) {
                vx = 0.0;
            }
            if (Math.abs(vy) < 1.0e-5) {
                vy = 0.0;
            }

            double pixelsPerInch = field.fieldWidth / 144.0;
            double ringRadiusPixels = RING_RADIUS_INCHES * pixelsPerInch;
            double px1 = x - ringRadiusPixels;
            double px2 = x + ringRadiusPixels;
            double py1 = y - ringRadiusPixels;
            double py2 = y + ringRadiusPixels;
            if (px1 <= field.X_MIN || px2 >= field.X_MAX || py1 <= field.Y_MIN || py2 >= field.Y_MAX) {
                // ring hit wall - note this doesn't handle possible deflections off the walls
                vx = 0.0;
                vy = 0.0;
                x = Math.max(field.X_MIN + ringRadiusPixels, Math.min(x, field.X_MAX - ringRadiusPixels));
                y = Math.max(field.Y_MIN + ringRadiusPixels, Math.min(y, field.Y_MAX - ringRadiusPixels));
                rolling = false;
            }
        }
    }

    @Override
    public synchronized void updateDisplay() {
        if (!onField) {
            y = controller.getField().Y_MAX + 15; // move ring off the field
        }
        super.updateDisplay();
    }

    @Override
    public VirtualBot getControlledBy() {
        return bot;
    }

    @Override
    public void setControlledBy(VirtualBot bot) {
        this.bot = bot;
    }

    public boolean isOnField() {
        return onField;
    }

    public void setOnField(boolean onField) {
        this.onField = onField;
    }

    public double getVx() {
        return vx;
    }

    public double getVy() {
        return vy;
    }

    public boolean isStationary() {
        return vx == 0.0 && vy == 0.0;
    }

    public void setVelocity(double vx, double vy) {
        this.vx = vx;
        this.vy = vy;
    }

    public boolean isInFlight() {
        return inFlight;
    }

    public void setInFlight(boolean inFlight) {
        this.inFlight = inFlight;
    }

    public boolean isRolling() {
        return rolling;
    }

    public void setRolling(boolean rolling) {
        this.rolling = rolling;
    }
}
