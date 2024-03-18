package org.murraybridgebunyips.bunyipslib.pid;

/**
 * A PIDF controller with no feedforward or integral.
 * <a href="https://github.com/FTCLib/FTCLib/blob/cedc52cee1bb549324c1ca5059c5feec3c054902/core/src/main/java/com/arcrobotics/ftclib/controller/PDController.java">Source</a>
 */
public class PDController extends PIDController {
    /**
     * Default constructor with just the coefficients
     *
     * @param kp The value of kP for the coefficients.
     * @param kd The value of kD for the coefficients.
     */
    public PDController(double kp, double kd) {
        super(kp, 0, kd);
    }

    /**
     * The extended constructor.
     *
     * @param kp The value of kP for the coefficients.
     * @param kd The value of kD for the coefficients.
     * @param sp The setpoint for the controller.
     * @param pv The process variable for the controller.
     */
    public PDController(double kp, double kd, double sp, double pv) {
        super(kp, 0, kd, sp, pv);
    }
}
