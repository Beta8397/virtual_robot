package org.murraybridgebunyips.bunyipslib.external;

/**
 * Marker interface that represents a control algorithm (PID, feedforward, etc).
 * <p>
 * This interface may not always represent the same behaviour in different contexts, such as open-loop and closed
 * loop control. Therefore, it is important to validate which controllers are being used within a control system
 * as the protection of type-safety may cause an invalid controller to be used against one that expects a certain
 * type of response the controller is not able to provide.
 *
 * @author Lucas Bubner, 2024
 */
public interface SystemController {
    /**
     * Sets the coefficients for this controller.
     *
     * @param coeffs a list of coefficients to set this controller to
     */
    void setCoefficients(double... coeffs);

    /**
     * Calculate the next output of this control algorithm.
     *
     * @param process  the current value or first setpoint
     * @param setpoint the expected value or second setpoint
     * @return process variable that should be applied to the system
     */
    double calculate(double process, double setpoint);

    /**
     * Reset this controller back to an un-accumulated state, if applicable.
     */
    void reset();
}
