package org.murraybridgebunyips.bunyipslib.external;

import org.murraybridgebunyips.bunyipslib.EncoderTicks;
import org.murraybridgebunyips.bunyipslib.external.ff.ArmFeedforward;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Velocity;

import java.util.function.Supplier;

/**
 * A composite PID and Feedforward controller that represents an arm suspended at an angle from gravity. This class
 * is similar to {@link PIDFFController}, but uses suppliers for angles to be used in calculating feedforward, allowing
 * proper calculation of the feedforward cosine coefficient.
 *
 * @author Lucas Bubner, 2024
 * @see EncoderTicks
 * @see PIDFFController
 */
public class ArmController implements SystemController {
    private final PIDController pid;
    private final ArmFeedforward ff;
    private final Supplier<Measure<Angle>> setPointAngleProvider;
    private final Supplier<Measure<Velocity<Angle>>> velocityAngleProvider;
    private final Supplier<Measure<Velocity<Velocity<Angle>>>> accelerationAngleProvider;

    /**
     * Construct a new ArmController.
     *
     * @param pid                       the PID controller to use
     * @param ff                        the ArmFeedforward controller to use
     * @param setPointAngleProvider     the setpoint angle provider
     * @param velocityAngleProvider     the angular velocity provider
     * @param accelerationAngleProvider the angular acceleration provider
     */
    public ArmController(PIDController pid, ArmFeedforward ff, Supplier<Measure<Angle>> setPointAngleProvider, Supplier<Measure<Velocity<Angle>>> velocityAngleProvider, Supplier<Measure<Velocity<Velocity<Angle>>>> accelerationAngleProvider) {
        this.pid = pid;
        this.ff = ff;
        this.setPointAngleProvider = setPointAngleProvider;
        this.velocityAngleProvider = velocityAngleProvider;
        this.accelerationAngleProvider = accelerationAngleProvider;
    }

    @Override
    public void setCoefficients(double... coeffs) {
        if (coeffs.length != 7) {
            throw new IllegalArgumentException("expected 7 coefficients, got " + coeffs.length);
        }
        pid.setPID(coeffs[0], coeffs[1], coeffs[2]);
        ff.setCoefficients(coeffs[3], coeffs[4], coeffs[5], coeffs[6]);
    }

    @Override
    public double calculate(double current, double target) {
        return pid.calculate(current, target) + ff.calculate(setPointAngleProvider.get(), velocityAngleProvider.get(), accelerationAngleProvider.get());
    }

    @Override
    public void reset() {
        pid.reset();
    }
}
