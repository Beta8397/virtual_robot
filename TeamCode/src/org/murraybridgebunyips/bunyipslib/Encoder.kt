package org.murraybridgebunyips.bunyipslib

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

/**
 * A generic Encoder, which is used to track the position of a motor in both relative and absolute scopes.
 * Includes methods for tracking, resetting, and calculating distance travelled.
 *
 * @author Lucas Bubner, 2023
 */
class Encoder(
    private val motor: DcMotorEx,
    private val ticksPerRevolution: Double? = null,
    private val wheelDiameterMM: Double? = null,
    private var reduction: Double = 1.0
) : ScopedEncoder {
    constructor(motor: DcMotorEx, ticksPerRevolution: Double) : this(motor, ticksPerRevolution, null)
    constructor(motor: DcMotorEx, ticksPerRevolution: Double, wheelDiameterMM: Double?) : this(
        motor,
        ticksPerRevolution,
        wheelDiameterMM,
        1.0
    )

    /**
     * Store a snapshot of encoder position when tracking is started.
     */
    var snapshot: Double = 0.0
        private set

    /**
     * Enable encoder and start tracking in the selected mode, which will also save a snapshot of the encoder position for relative tracking.
     * @param mode the mode to track the encoder in
     */
    override fun track(mode: DcMotor.RunMode) {
        // Store the current encoder position
        this.snapshot = motor.currentPosition.toDouble()
        motor.mode = mode
    }

    /**
     * Hold the current position of the encoder using RUN_TO_POSITION.
     * @param holdingPower the power to hold the position at
     */
    fun holdCurrentPosition(holdingPower: Double) {
        motor.targetPosition = motor.currentPosition
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.power = holdingPower
    }

    /**
     * Reset encoder positions to zero. Useful when saved state is not needed or can be discarded.
     * This will also stop the motor.
     */
    override fun reset() {
        this.snapshot = 0.0
        motor.power = 0.0
        val prev = motor.mode
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.targetPosition = 0
        motor.mode = prev
    }

    /**
     * Get a movement reading in ticks from the encoder since the last track()
     * Can use an optional parameter to use since reset() position instead of track()
     * @return encoder value relative to last track() call, or since the last reset() call
     */
    override fun position(scope: ScopedEncoder.Scope): Double {
        return if (scope == ScopedEncoder.Scope.RELATIVE) {
            (motor.currentPosition.toDouble() - snapshot) * reduction
        } else {
            motor.currentPosition.toDouble() * reduction
        }
    }

    /**
     * Get the number of revolutions the encoder has travelled since the last track()
     * Can use an optional parameter to use since reset() position instead of track()
     * @return revolutions indicating how far the encoder has travelled
     */
    override fun travelledRevolutions(scope: ScopedEncoder.Scope): Double {
        // Equation: encoder ticks / ticksPerRevolution
        if (ticksPerRevolution == null) {
            throw IllegalStateException("Odometer: ticksPerRevolution must be set to use travelledRevolutions()")
        }
        // Return travelled revolutions depending on selected accuracy
        return position(scope) / ticksPerRevolution
    }

    /**
     * Get the distance travelled by the encoder since the last track()
     * Can use an optional parameter to use since reset() position instead of track()
     * @return millimetres indicating how far the encoder has travelled
     */
    override fun travelledMM(scope: ScopedEncoder.Scope): Double {
        // Equation: circumference (2*pi*r) * (encoder ticks / ticksPerRevolution)
        if (wheelDiameterMM == null || ticksPerRevolution == null) {
            throw IllegalStateException("Odometer: wheelDiameterMM and ticksPerRevolution must be set to use travelledMM()")
        }
        // Return travelled distance in millimetres depending on selected accuracy
        return Math.PI * wheelDiameterMM * (position(scope) / ticksPerRevolution)
    }
}