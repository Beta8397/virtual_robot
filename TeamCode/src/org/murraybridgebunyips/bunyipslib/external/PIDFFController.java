package org.murraybridgebunyips.bunyipslib.external;

import org.murraybridgebunyips.bunyipslib.Encoder;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDController;

import java.util.Arrays;

/**
 * A composite controller to use for PID/FF controls (with kV/kA calculated components) for error calculation.
 * <p>
 * To use an ArmFeedforward, consider using the {@link ArmController}, which will properly integrate the cosine of
 * the arm position as part of your feedforward during a RUN_TO_POSITION.
 *
 * @author Lucas Bubner, 2024
 * @see ArmController
 */
public class PIDFFController implements SystemController {
    private final PIDController pid;
    private final Encoder encoder;
    private final SystemController ff;

    /**
     * Construct a new PIDFFController.
     *
     * @param pid     the PID controller to use
     * @param encoder the encoder to retrieve velocity/acceleration information from
     * @param ff      the kV/kA feedforward to use
     */
    public PIDFFController(PIDController pid, Encoder encoder, SystemController ff) {
        this.pid = pid;
        this.encoder = encoder;
        this.ff = ff;
    }

    @Override
    public void setCoefficients(double... coeffs) {
        if (coeffs.length < 3) {
            throw new IllegalArgumentException("expected >=3 arguments, got " + coeffs.length);
        }
        pid.setPID(coeffs[0], coeffs[1], coeffs[2]);
        ff.setCoefficients(Arrays.stream(coeffs).skip(3).toArray());
    }

    @Override
    public double calculate(double current, double target) {
        return pid.calculate(current, target) + ff.calculate(encoder.getVelocity(), encoder.getAcceleration());
    }

    @Override
    public void reset() {
        pid.reset();
        ff.reset();
    }
}
