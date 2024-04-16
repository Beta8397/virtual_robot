package org.murraybridgebunyips.bunyipslib

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorImpl
import org.murraybridgebunyips.bunyipslib.external.units.Distance
import org.murraybridgebunyips.bunyipslib.external.units.Measure
import org.murraybridgebunyips.bunyipslib.external.units.Units.Millimeters

/**
 * A generic encoder based motor, which is used to track the position of a motor in both relative and absolute scopes.
 * Includes methods for tracking, resetting, and calculating distance travelled.
 *
 * @author Lucas Bubner, 2023
 */
class EncoderMotor @JvmOverloads constructor(
    private val motor: DcMotorEx,
    private var reduction: Double = 1.0,
    private val ticksPerRevolution: Double? = null,
    private val wheelDiameter: Measure<Distance>? = null
) : DcMotorImpl(motor.controller, motor.portNumber), ScopedEncoder {
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
     * @param holdingPower the power to hold the position at, default is 1.0
     */
    @JvmOverloads
    fun holdCurrentPosition(holdingPower: Double = 1.0) {
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
     * @return distance indicating how far the encoder has travelled
     */
    override fun travelled(scope: ScopedEncoder.Scope): Measure<Distance> {
        // Equation: circumference (2*pi*r) * (encoder ticks / ticksPerRevolution)
        if (wheelDiameter == null || ticksPerRevolution == null) {
            throw IllegalStateException("Odometer: wheelDiameter and ticksPerRevolution must be set to use travelledMM()")
        }
        // Return travelled distance in millimetres depending on selected accuracy
        return Millimeters.of(Math.PI * wheelDiameter.inUnit(Millimeters) * (position(scope) / ticksPerRevolution))
    }
}