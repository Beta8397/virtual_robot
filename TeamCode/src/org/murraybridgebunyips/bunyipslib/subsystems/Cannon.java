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
    // Name of the cannon for telemetry
    private String NAME = "Cannon";
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
     * Set the name of the cannon to display in telemetry.
     *
     * @param newName the name to set
     * @return this
     */
    public Cannon withName(String newName) {
        NAME = newName;
        return this;
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
     * Fire the cannon.
     *
     * @return Fire cannon task
     */
    public Task fireTask() {
        return new RunTask(this::fire, this, true).withName("FireCannonTask");
    }

    /**
     * Reset the cannon.
     *
     * @return Reset cannon task
     */
    public Task resetTask() {
        return new RunTask(this::reset, this, true).withName("ResetCannonTask");
    }

    @Override
    protected void periodic() {
        opMode.addTelemetry("%: %", NAME, target == FIRED ? "FIRED" : "READY");
        prolong.setPosition(target);
    }
}
