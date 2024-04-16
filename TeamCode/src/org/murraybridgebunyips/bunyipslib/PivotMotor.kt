package org.murraybridgebunyips.bunyipslib

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorImpl
import org.murraybridgebunyips.bunyipslib.external.units.Angle
import org.murraybridgebunyips.bunyipslib.external.units.Distance
import org.murraybridgebunyips.bunyipslib.external.units.Measure
import org.murraybridgebunyips.bunyipslib.external.units.Units.Degrees
import org.murraybridgebunyips.bunyipslib.external.units.Units.Millimeters

/**
 * Extension of DcMotor that implements a pivotal encoder for tracking the position of a pivot.
 * @author Lucas Bubner, 2023
 */
class PivotMotor(
    private val motor: DcMotorEx,
    private val ticksPerRevolution: Double,
    private val reduction: Double = 1.0,
) : DcMotorImpl(motor.controller, motor.portNumber), ScopedEncoder {
    constructor(motor: DcMotorEx, ticksPerRevolution: Double) : this(motor, ticksPerRevolution, 1.0)

    private val encoderMotor: EncoderMotor = EncoderMotor(motor, 1.0, ticksPerRevolution)

    /**
     * Enable encoder and start tracking in the selected mode, which will also save a snapshot of the encoder position for relative tracking.
     * @param mode the mode to track the encoder in
     */
    override fun track(mode: DcMotor.RunMode) {
        encoderMotor.track(mode)
    }

    /**
     * Reset encoder positions to zero. Useful when saved state is not needed or can be discarded.
     * This will also stop the motor.
     */
    override fun reset() {
        encoderMotor.reset()
    }

    /**
     * Get a movement reading in ticks from the encoder since the last track()
     */
    override fun position(scope: ScopedEncoder.Scope): Double {
        return encoderMotor.position(scope)
    }

    /**
     * Get the distance travelled in revolutions.
     * @param scope the scope to calculate the distance in
     */
    override fun travelledRevolutions(scope: ScopedEncoder.Scope): Double {
        return encoderMotor.travelledRevolutions(scope)
    }

    /**
     * This method is not used for pivots, and will always return 0.0.
     * @param scope
     */
    override fun travelled(scope: ScopedEncoder.Scope): Measure<Distance> {
        // Wheel diameter is not used for pivots
        Dbg.error(Text.getCallingUserCodeFunction(), "Wheel diameter is not used for pivots")
        return Millimeters.zero()
    }

    /**
     * Hold the current position of the encoder using RUN_TO_POSITION.
     * @param holdingPower the power to hold the position at, default is 1.0
     */
    @JvmOverloads
    fun holdCurrentPosition(holdingPower: Double = 1.0) {
        encoderMotor.holdCurrentPosition(holdingPower)
    }

    /**
     * Get the current angle of the pivot.
     */
    @JvmOverloads
    fun getCurrent(scope: ScopedEncoder.Scope = ScopedEncoder.Scope.RELATIVE): Measure<Angle> {
        return Degrees.of((encoderMotor.position(scope) / ticksPerRevolution) * 360)
    }

    /**
     * Get the current target angle of the pivot.
     */
    fun getTarget(): Measure<Angle> {
        return Degrees.of(((motor.targetPosition + encoderMotor.snapshot.toInt()) / ticksPerRevolution) * 360)
    }

    /**
     * Set the target angle of the pivot.
     */
    fun set(angle: Measure<Angle>) {
        motor.targetPosition =
            ((angle.inUnit(Degrees) / 360) * ticksPerRevolution / reduction).toInt() - encoderMotor.snapshot.toInt()
    }
}