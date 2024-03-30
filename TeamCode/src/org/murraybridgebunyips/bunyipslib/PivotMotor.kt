package org.murraybridgebunyips.bunyipslib

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorImpl

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

    private val encoder: Encoder = Encoder(motor, ticksPerRevolution, null, reduction)

    /**
     * Enable encoder and start tracking in the selected mode, which will also save a snapshot of the encoder position for relative tracking.
     * @param mode the mode to track the encoder in
     */
    override fun track(mode: DcMotor.RunMode) {
        encoder.track(mode)
    }

    /**
     * Reset encoder positions to zero. Useful when saved state is not needed or can be discarded.
     * This will also stop the motor.
     */
    override fun reset() {
        encoder.reset()
    }

    /**
     * Get a movement reading in ticks from the encoder since the last track()
     */
    override fun position(scope: ScopedEncoder.Scope): Double {
        return encoder.position(scope)
    }

    /**
     * Get the distance travelled in revolutions.
     * @param scope the scope to calculate the distance in
     */
    override fun travelledRevolutions(scope: ScopedEncoder.Scope): Double {
        return encoder.travelledRevolutions(scope)
    }

    /**
     * Get the distance travelled in millimeters.
     * This method is not used for pivots, and will always return 0.0.
     * @param scope the scope to calculate the distance in
     */
    override fun travelledMM(scope: ScopedEncoder.Scope): Double {
        // Wheel diameter is not used for pivots
        Dbg.error(Text.getCallingUserCodeFunction(), "Wheel diameter is not used for pivots")
        return 0.0
    }

    /**
     * Hold the current position of the encoder using RUN_TO_POSITION.
     * @param holdingPower the power to hold the position at
     */
    fun holdCurrentPosition(holdingPower: Double) {
        encoder.holdCurrentPosition(holdingPower)
    }

    /**
     * Get the current degrees of the pivot.
     */
    fun getDegrees(scope: ScopedEncoder.Scope = ScopedEncoder.Scope.RELATIVE): Double {
        return (encoder.position(scope) / ticksPerRevolution) * 360
    }

    /**
     * Get the current degrees of the pivot.
     */
    // Java interop
    fun getDegrees(): Double {
        return getDegrees(ScopedEncoder.Scope.RELATIVE)
    }

    /**
     * Get the current radians of the pivot.
     */
    fun getRadians(scope: ScopedEncoder.Scope = ScopedEncoder.Scope.RELATIVE): Double {
        return Math.toRadians(getDegrees(scope))
    }

    /**
     * Get the current radians of the pivot.
     */
    // Java interop
    fun getRadians(): Double {
        return getRadians(ScopedEncoder.Scope.RELATIVE)
    }

    /**
     * Set the target position of the pivot in degrees.
     */
    fun setDegrees(degrees: Double) {
        motor.targetPosition =
            ((degrees / 360) * ticksPerRevolution / reduction).toInt() - encoder.snapshot.toInt()
    }

    /**
     * Set the target position of the pivot in radians.
     */
    fun setRadians(radians: Double) {
        setDegrees(Math.toDegrees(radians))
    }
}