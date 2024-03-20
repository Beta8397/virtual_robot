package org.murraybridgebunyips.bunyipslib;

import com.qualcomm.robotcore.hardware.Servo;

import org.murraybridgebunyips.bunyipslib.tasks.CallbackTask;
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
    // NOTE: Servos go from 1 to 0, 1 being right as set on the servo programmer and vice versa.
    private static final double FIRED = 1.0;
    private static final double RESET = 0.0;
    private final Servo prolong;
    private double target;

    /**
     * Constructs a new Cannon.
     *
     * @param prolong the servo to use
     */
    public Cannon(Servo prolong) {
        assertParamsNotNull(prolong);
        this.prolong = prolong;

        // We assume there will always be something in the reset position for us to hold
        target = RESET;
        update();
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
        return new CallbackTask(this::fire, this, true).withName("FireCannonTask");
    }

    /**
     * Reset the cannon.
     *
     * @return Reset cannon task
     */
    public Task resetTask() {
        return new CallbackTask(this::reset, this, true).withName("ResetCannonTask");
    }

    @Override
    protected void periodic() {
        opMode.addTelemetry("Cannon: %", target == FIRED ? "FIRED" : "READY");
        prolong.setPosition(target);
    }
}
