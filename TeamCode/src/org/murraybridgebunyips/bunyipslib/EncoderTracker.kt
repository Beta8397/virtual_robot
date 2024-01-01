package org.murraybridgebunyips.bunyipslib

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

/**
 * Interface abstraction for encoder motors, for functionality such as enabling/disabling tracking.
 * Includes calculations for distance travelled and revolutions turned.
 * @author Lucas Bubner, 2023
 */
interface EncoderTracker {
    enum class Scope {
        RELATIVE, GLOBAL
    }

    val wheelDiameterMM: Double?
    val ticksPerRevolution: Double?
    val motor: DcMotorEx

    /**
     * Reduction factor of the motor, used in gearing calculations.
     */
    var reduction: Double

    /**
     * Store a snapshot of encoder position when tracking is started.
     */
    var snapshot: Double

    /**
     * Enable encoder and start tracking, which will also save a snapshot of the encoder position
     */
    fun track() {
        // Store the current encoder position
        this.snapshot = motor.currentPosition.toDouble()
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    /**
     * Reset encoder positions to zero. Useful when saved state is not needed or can be discarded.
     */
    fun reset() {
        this.snapshot = 0.0
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    /**
     * Get a movement reading in ticks from the encoder since the last track()
     * Can use an optional parameter to use since reset() position instead of track()
     * @return encoder value relative to last track() call, or since the last reset() call
     */
    fun position(scope: Scope = Scope.RELATIVE): Double {
        return if (scope == Scope.RELATIVE) {
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
    fun travelledRevolutions(scope: Scope = Scope.RELATIVE): Double {
        // Equation: encoder ticks / ticksPerRevolution
        if (ticksPerRevolution == null) {
            throw IllegalStateException("Odometer: ticksPerRevolution must be set to use travelledRevolutions()")
        }
        // Return travelled revolutions depending on selected accuracy
        return position(scope) / ticksPerRevolution!!
    }

    /**
     * Get the distance travelled by the encoder since the last track()
     * Can use an optional parameter to use since reset() position instead of track()
     * @return millimetres indicating how far the encoder has travelled
     */
    fun travelledMM(scope: Scope = Scope.RELATIVE): Double {
        // Equation: circumference (2*pi*r) * (encoder ticks / ticksPerRevolution)
        if (wheelDiameterMM == null || ticksPerRevolution == null) {
            throw IllegalStateException("Odometer: wheelDiameterMM and ticksPerRevolution must be set to use travelledMM()")
        }
        // Return travelled distance in millimetres depending on selected accuracy
        return Math.PI * wheelDiameterMM!! * (position(scope) / ticksPerRevolution!!)
    }
}