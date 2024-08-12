package org.murraybridgebunyips.bunyipslib.external;

import org.murraybridgebunyips.bunyipslib.external.ff.SimpleMotorFeedforward;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDController;

import java.util.function.DoubleSupplier;

/**
 * A generic velocity controller with PID and feedforward components.
 */
public class VelocityFFController implements SystemController {
    private final PIDController pid;
    private final SimpleMotorFeedforward ff;

    private final DoubleSupplier acceleration;
    private final double bufferFraction;
    private final double maxMotorTicksPerSecond;

    /**
     * Create a new VelocityFFController.
     *
     * @param pid                    the PID controller to use
     * @param feedforward            the feedforward controller to use
     * @param accelerationProvider   the supplier for acceleration data
     * @param bufferFraction         the fractional value to use for velocity control, range (0, 1]
     * @param maxMotorTicksPerSecond maximum achievable ticks per second (see your motor configuration)
     */
    public VelocityFFController(PIDController pid, SimpleMotorFeedforward feedforward, DoubleSupplier accelerationProvider, double bufferFraction, double maxMotorTicksPerSecond) {
        ff = feedforward;
        acceleration = accelerationProvider;
        this.pid = pid;
        this.maxMotorTicksPerSecond = maxMotorTicksPerSecond;
        if (bufferFraction <= 0 || bufferFraction > 1) {
            throw new IllegalArgumentException("bufferFraction exceeds domain (0, 1]");
        }
        this.bufferFraction = bufferFraction;
    }

    @Override
    public void setCoefficients(double... coeffs) {
        if (coeffs.length != 6) {
            throw new IllegalArgumentException("expected 6 arguments, got " + coeffs.length);
        }
        pid.setPID(coeffs[0], coeffs[1], coeffs[2]);
        ff.setCoefficients(coeffs[3], coeffs[4], coeffs[5]);
    }

    @Override
    public double calculate(double vel, double power) {
        double speed = bufferFraction * power * maxMotorTicksPerSecond;
        double velocity = pid.calculate(vel, speed) + ff.calculate(speed, acceleration.getAsDouble());
        return velocity / maxMotorTicksPerSecond;
    }

    @Override
    public void reset() {
        pid.reset();
    }
}
