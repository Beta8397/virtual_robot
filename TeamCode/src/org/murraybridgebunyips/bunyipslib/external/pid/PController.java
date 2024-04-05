package org.murraybridgebunyips.bunyipslib.external.pid;

/**
 * A PIDF controller with no feedforward, integral, or derivative.
 * <a href="https://github.com/FTCLib/FTCLib/blob/feature-new-pure-pursuit/core/src/main/java/com/arcrobotics/ftclib/controller/PController.java">Source</a>
 */
public class PController extends PDController {
    /**
     * Default constructor, only takes a p-value.
     *
     * @param kp The value of kP for the coefficients.
     */
    public PController(double kp) {
        super(kp, 0);
    }

    /**
     * The extended constructor.
     *
     * @param kp The value of kP for the coefficients.
     * @param sp The setpoint for the controller.
     * @param pv The process variable for the controller.
     */
    public PController(double kp, double sp, double pv) {
        super(kp, 0, sp, pv);
    }
}