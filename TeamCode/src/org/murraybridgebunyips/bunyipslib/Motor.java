package org.murraybridgebunyips.bunyipslib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.murraybridgebunyips.bunyipslib.external.SystemController;
import org.murraybridgebunyips.bunyipslib.external.VelocityFFController;
import org.murraybridgebunyips.bunyipslib.external.ff.SimpleMotorFeedforward;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDFController;

import java.util.ArrayList;

/**
 * Wrapper class for a {@link DcMotor} that uses custom control algorithms to operate {@link RunMode#RUN_USING_ENCODER}
 * and {@link RunMode#RUN_TO_POSITION} modes. Internally integrates a gain scheduler to allow for more precise
 * system coefficients against gravity and other external forces.
 * <p>
 * This class is designed to be down casted to a DcMotor where used, integrating with the current motor modes while
 * providing faster, predictable control systems.
 *
 * @author Lucas Bubner, 2024
 */
public class Motor extends DcMotorImplEx {
    private final ArrayList<InterpolatedLookupTable> rtpGains = new ArrayList<>();
    private final ArrayList<InterpolatedLookupTable> rueGains = new ArrayList<>();
    private final Encoder encoder;
    private SystemController rtpController;
    private SystemController rueController;
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;

    // TODO: virtual robot will not work with this class

    /**
     * Wrap a DcMotor to use in the Motor class.
     *
     * @param motor the DcMotor from hardwareMap to use.
     */
    public Motor(DcMotorEx motor) {
        super(motor.getController(), motor.getPortNumber(), motor.getDirection(), motor.getMotorType());
        // The actual motor should *always* be running in RUN_WITHOUT_ENCODER
        super.setMode(RunMode.RUN_WITHOUT_ENCODER);
        encoder = new Encoder(super::getCurrentPosition, super::getVelocity);
        setTargetPosition(getCurrentPosition());
    }

    /**
     * Call to use encoder overflow (exceeding 32767 ticks/sec) correction on {@link #getVelocity()}.
     */
    public void useEncoderOverflowCorrection() {
        encoder.useEncoderOverflowCorrection();
    }

    /**
     * Set a system controller to use for {@link RunMode#RUN_TO_POSITION}.
     * <p>
     * The coefficients of this controller can be modified through {@link #scheduleRunToPositionGains()}.
     *
     * @param controller the controller to use, recommended to use a closed-loop controller such as PID
     */
    public void setRunToPositionController(SystemController controller) {
        rtpController = controller;
    }

    /**
     * Call to build a list of encoder tick positions where you want your {@link RunMode#RUN_TO_POSITION} system controller
     * gains to be. When this builder is built with {@code build()}, it will interpolate between each value to provide a
     * continuous PID range that will be used when {@link #setPower(double)} is called.
     * <p>
     * Note that when using a motor with this class, the PID coefficients attached to the motor itself will be used only
     * if another controller is not specified, and will only take a <b>snapshot at runtime</b> of these values to populate
     * this controller.
     *
     * @return a builder to specify encoder tick positions to gains of your {@link RunMode#RUN_TO_POSITION} controller
     */
    public GainScheduling scheduleRunToPositionGains() {
        rtpGains.clear();
        return new GainScheduling(RunMode.RUN_TO_POSITION);
    }

    /**
     * Call to build a list of encoder tick positions where you want your {@link RunMode#RUN_USING_ENCODER} system controller
     * gains to be. When this builder is built with {@code build()}, it will interpolate between each value to provide a
     * continuous PID range that will be used when {@link #setPower(double)} is called.
     *
     * @return a builder to specify encoder tick positions to gains of your {@link RunMode#RUN_USING_ENCODER} controller
     */
    public GainScheduling scheduleRunUsingEncoderGains() {
        rueGains.clear();
        return new GainScheduling(RunMode.RUN_USING_ENCODER);
    }

    /**
     * Set a system controller to use for {@link RunMode#RUN_USING_ENCODER}.
     * <p>
     * The coefficients of this controller can be modified through {@link #scheduleRunUsingEncoderGains()}.
     *
     * @param controller the controller to use, recommended to use a velocity controller (PID+FF) such as VelocityFF.
     */
    public void setRunUsingEncoderController(SystemController controller) {
        rueController = controller;
    }

    /**
     * Reset the encoder value without stopping the motor. Will internally be called if the motor is attempted
     * to be set to {@link RunMode#STOP_AND_RESET_ENCODER}.
     */
    public void resetEncoder() {
        encoder.reset();
    }

    /**
     * Get the encoder attached to this motor.
     *
     * @return the encoder object that is used for encoder readings on this motor
     */
    public Encoder getEncoder() {
        return encoder;
    }

    /**
     * @return the estimated acceleration of this motor.
     */
    public double getAcceleration() {
        return encoder.getAcceleration();
    }

    /**
     * @return the current position in ticks of this motor, with velocity estimation.
     */
    @Override
    public int getCurrentPosition() {
        return encoder.getPosition();
    }

    /**
     * @return the current velocity of this motor as specified by your settings
     */
    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * This method is not supported.
     *
     * @inheritDoc
     */
    @Deprecated
    @Override
    public void setVelocity(double vel) {
        throw new UnsupportedOperationException("Unsupported on a BunyipsLib motor object. Use RUN_USING_ENCODER and setPower().");
    }

    /**
     * Modified version of {@code setMode} where the modes will never actually be propagated to the motors, and instead
     * managed internally by the modified {@link #setPower(double)} method. The actual motor object will always be in
     * {@link RunMode#RUN_WITHOUT_ENCODER}.
     *
     * @param mode the new current run mode for this motor
     */
    @Override
    public void setMode(RunMode mode) {
        if (mode == RunMode.STOP_AND_RESET_ENCODER) {
            setPower(0);
            resetEncoder();
            return;
        }
        this.mode = mode;
    }

    /**
     * Note that this method will try to access the {@link RunMode#RUN_TO_POSITION} controller to access information there. This
     * is otherwise unsupported and you should try to access the actual controller to see this information, unless
     * you are downcasting in which this will assume you are using a PID controller.
     *
     * @inheritDoc
     */
    @Deprecated
    @Override
    public int getTargetPositionTolerance() {
        // Built-in support for PIDF as it is the most common use case for RTP
        if (rtpController != null && rtpController instanceof PIDFController) {
            return (int) Math.ceil(((PIDFController) rtpController).getTolerance()[0]);
        }
        throw new UnsupportedOperationException("Can't access target position information on the currently used RTP controller. It may be the case that this controller is open-loop, or not a PID controller, as any tolerance configuration should be modified by your controller, not by this method.");
    }

    /**
     * Note that this method will try to access the {@link RunMode#RUN_TO_POSITION} controller to access information there. This
     * is otherwise unsupported and you should try to access the actual controller to see this information, unless
     * you are downcasting in which this will assume you are using a PID controller.
     *
     * @inheritDoc
     */
    @Deprecated
    @Override
    public void setTargetPositionTolerance(int tolerance) {
        if (rtpController != null && rtpController instanceof PIDFController) {
            ((PIDFController) rtpController).setTolerance(tolerance);
        }
        throw new UnsupportedOperationException("Can't access target position information on the currently used RTP controller. It may be the case that this controller is open-loop, or not a PID controller, as any tolerance configuration should be modified by your controller, not by this method.");
    }

    /**
     * Note that this method will try to access the {@link RunMode#RUN_TO_POSITION} controller to access information there. This
     * is otherwise unsupported and you should try to access the actual controller to see this information, unless
     * you are downcasting in which this will assume you are using a PID controller.
     *
     * @inheritDoc
     */
    @Deprecated
    @Override
    public boolean isBusy() {
        if (rtpController != null && rtpController instanceof PIDFController) {
            return mode == RunMode.RUN_TO_POSITION && !((PIDFController) rtpController).atSetPoint();
        }
        throw new UnsupportedOperationException("Can't access target position information on the currently used RTP controller. It may be the case that this controller is open-loop, or not a PID controller, as any tolerance configuration should be modified by your controller, not by this method.");
    }

    /**
     * This method is not supported.
     *
     * @inheritDoc
     */
    @Deprecated
    @Override
    public void setVelocity(double vel, AngleUnit unit) {
        throw new UnsupportedOperationException("Unsupported on a BunyipsLib motor object. Use RUN_USING_ENCODER and setPower().");
    }

    /**
     * Update system controllers and propagate new power levels to the motor. Should be called frequently.
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     */
    @Override
    public void setPower(double power) {
        switch (mode) {
            case RUN_TO_POSITION:
                if (rtpController == null) {
                    PIDFCoefficients coeffs = getPIDFCoefficients(RunMode.RUN_TO_POSITION);
                    Dbg.warn("[%] No RUN_TO_POSITION controller was specified. This motor will be using the default PIDF coefficients to create a fallback PIDF controller with values from %. Please set your own controller.", getDeviceName(), coeffs);
                    rtpController = new PIDFController(coeffs.p, coeffs.i, coeffs.d, coeffs.f);
                }
                if (!rtpGains.isEmpty())
                    rtpController.setCoefficients(rtpGains.stream().mapToDouble((g) -> g.get(getCurrentPosition())).toArray());
                // In a RUN_TO_POSITION context, the controller is used for error correction, which will multiply the
                // allowed power by the user against the encoder error by your (usually PID) controller.
                super.setPower(Math.abs(power) * rtpController.calculate(getCurrentPosition(), getTargetPosition()));
                break;
            case RUN_USING_ENCODER:
                if (rueController == null) {
                    PIDFCoefficients coeffs = getPIDFCoefficients(RunMode.RUN_USING_ENCODER);
                    Dbg.warn("[%] No RUN_USING_ENCODER controller was specified. This motor will be using the default PIDF coefficients to create a fallback PID and static FF controller with values from %. Please set your own controller.", getDeviceName(), coeffs);
                    PIDController pid = new PIDController(coeffs.p, coeffs.i, coeffs.d);
                    rueController = new VelocityFFController(pid, new SimpleMotorFeedforward(coeffs.f, 0, 0), encoder::getAcceleration, 1.0, motorType.getAchieveableMaxTicksPerSecond());
                }
                if (!rueGains.isEmpty())
                    rueController.setCoefficients(rueGains.stream().mapToDouble((g) -> g.get(getCurrentPosition())).toArray());
                // In RUN_USING_ENCODER, the controller is expected to take in the encoder velocity and target power,
                // which usually consists internally of a PID and feedforward controller.
                super.setPower(rueController.calculate(getVelocity(), power));
                break;
            case RUN_WITHOUT_ENCODER:
                super.setPower(power);
                break;
        }
    }

    /**
     * Gain scheduler builder which will interpolate on build and populate gains. Not to be used as-is.
     *
     * @see #scheduleRunToPositionGains()
     * @see #scheduleRunUsingEncoderGains()
     */
    public class GainScheduling {
        private final ArrayList<InterpolatedLookupTable> gains;

        private GainScheduling(RunMode targetMode) {
            gains = targetMode == RunMode.RUN_TO_POSITION ? rtpGains : rueGains;
        }

        /**
         * Specify a position in encoder ticks that you want the gains of your system controller to be.
         * This should be used with known values, such as knowing for 0 ticks the gains should be 1,0,0, etc.
         * <p>
         * The more positions that are added to this builder, the more accurate your final gain scheduling model will be.
         *
         * @param positionTicks the position in encoder ticks that the system controller coefficients should be
         * @param coeffs        the coefficients at encoder ticks position
         * @return the builder
         */
        public GainScheduling atPosition(double positionTicks, double... coeffs) {
            for (int i = 0; i < coeffs.length; i++) {
                if (gains.size() <= i) {
                    gains.add(new InterpolatedLookupTable());
                }
                gains.get(i).add(positionTicks, coeffs[i]);
            }
            return this;
        }

        /**
         * Create the interpolated lookup tables for use in gains scheduling.
         */
        public void build() {
            for (int i = 0; i < gains.size(); i++) {
                gains.get(i).createLUT();
            }
        }
    }
}
