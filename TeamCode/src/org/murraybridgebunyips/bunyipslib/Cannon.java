package org.murraybridgebunyips.bunyipslib;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class for the paper plane launcher.
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

    public Cannon(Servo prolong) {
        this.prolong = prolong;

        // We assume there will always be a paper plane in the cannon at the start of a match
        target = RESET;
        update();
    }

    /**
     * Fire in the hole!
     */
    public void fire() {
        target = FIRED;
    }

    /**
     * Reset the cannon to its initial position
     */
    public void reset() {
        target = RESET;
    }

    public void update() {
        opMode.addTelemetry("Cannon: %", target == FIRED ? "FIRED" : "READY");
        prolong.setPosition(target);
    }
}
