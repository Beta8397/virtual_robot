package org.murraybridgebunyips.bunyipslib.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * Class for a generic servo switch.
 * A whole new file is technically unnecessary, but we wanted to make a class named Cannon
 * <p></p>
 * "Fire in the hole!"
 *
 * @author Lachlan Paul, 2023
 * @author Lucas Bubner, 2023
 */
public class Cannon extends BunyipsSubsystem {
    private final double FIRED;
    private final double RESET;
    private Servo prolong;
    private double target;

    /**
     * Constructs a new Cannon.
     *
     * @param prolong       the servo to use
     * @param closePosition the position to set the servo to when not firing
     * @param openPosition  the position to set the servo to when firing
     */
    public Cannon(Servo prolong, double openPosition, double closePosition) {
        if (openPosition == closePosition)
            throw new IllegalArgumentException("Open and close positions cannot be the same");

        FIRED = openPosition;
        RESET = closePosition;

        if (!assertParamsNotNull(prolong)) return;
        this.prolong = prolong;

        // We assume there will always be something in the reset position for us to hold
        target = RESET;
        update();
    }

    /**
     * Constructs a new Cannon.
     * Implicitly set the close position to 0.0 and the open position to 1.0.
     *
     * @param prolong the servo to use
     */
    public Cannon(Servo prolong) {
        this(prolong, 1.0, 0.0);
    }

    /**
     * Fire in the hole!
     *
     * @return this
     */
    public Cannon fire() {
        target = FIRED;
        return this;
    }

    /**
     * Reset the cannon to its initial position
     *
     * @return this
     */
    public Cannon reset() {
        target = RESET;
        return this;
    }

    /**
     * Query if the cannon is fired.
     *
     * @return true if the cannon is fired
     */
    public boolean isFired() {
        return target == FIRED;
    }

    /**
     * Query if the cannon is reset.
     *
     * @return true if the cannon is reset
     */
    public boolean isReset() {
        return target == RESET;
    }

    /**
     * Fire the cannon.
     *
     * @return Fire cannon task
     */
    public Task fireTask() {
        return new RunTask(this::fire).onSubsystem(this, true).withName("Fire");
    }

    /**
     * Reset the cannon.
     *
     * @return Reset cannon task
     */
    public Task resetTask() {
        return new RunTask(this::reset).onSubsystem(this, true).withName("Reset");
    }

    @Override
    protected void periodic() {
        opMode.telemetry.add("%: %", name, target == FIRED ? "<font color='red'><b>FIRED</b></font>" : "<font color='green'>READY</font>");
        prolong.setPosition(target);
    }
}
