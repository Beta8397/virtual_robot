package org.murraybridgebunyips.bunyipslib

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorImpl

/**
 * Extension of DcMotor that implements a pivotal encoder for tracking the position of a pivot.
 * @author Lucas Bubner, 2023
 */
class PivotMotor(
    override val motor: DcMotorEx,
    override val ticksPerRevolution: Double,
    override var reduction: Double = 1.0,
) : DcMotorImpl(motor.controller, motor.portNumber), EncoderTracker {
    override val wheelDiameterMM = null
    override var snapshot: Double = 0.0

    override fun travelledMM(scope: EncoderTracker.Scope): Double {
        throw IllegalAccessException("PivotMotor: Cannot access travelledMM on a PivotMotor or PivotMotor variant")
    }

    /**
     * Setup the motor for tracking the position of a target position.
     */
    fun setup() {
        motor.targetPosition = motor.currentPosition
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun getDegrees(scope: EncoderTracker.Scope = EncoderTracker.Scope.RELATIVE): Double {
        return (position(scope) / ticksPerRevolution) * 360
    }

    // Java interop
    fun getDegrees(): Double {
        return getDegrees(EncoderTracker.Scope.RELATIVE)
    }

    fun getRadians(scope: EncoderTracker.Scope = EncoderTracker.Scope.RELATIVE): Double {
        return (position(scope) / ticksPerRevolution) * (2 * Math.PI)
    }

    // Java interop
    fun getRadians(): Double {
        return getRadians(EncoderTracker.Scope.RELATIVE)
    }

    fun setDegrees(degrees: Double) {
        motor.targetPosition =
            ((degrees / 360) * ticksPerRevolution / reduction).toInt() - snapshot.toInt()
    }

    fun setRadians(radians: Double) {
        motor.targetPosition =
            ((radians / (2 * Math.PI)) * ticksPerRevolution / reduction).toInt() - snapshot.toInt()
    }
}