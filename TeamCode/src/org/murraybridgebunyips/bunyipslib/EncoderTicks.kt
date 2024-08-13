package org.murraybridgebunyips.bunyipslib

import com.qualcomm.robotcore.hardware.DcMotorEx
import org.murraybridgebunyips.bunyipslib.external.units.Angle
import org.murraybridgebunyips.bunyipslib.external.units.Distance
import org.murraybridgebunyips.bunyipslib.external.units.Measure
import org.murraybridgebunyips.bunyipslib.external.units.Units.Degrees
import org.murraybridgebunyips.bunyipslib.external.units.Units.Meters
import org.murraybridgebunyips.bunyipslib.external.units.Units.Second
import org.murraybridgebunyips.bunyipslib.external.units.Velocity

/**
 * Utility class for converting between SI units and encoder ticks.
 *
 * @author Lucas Bubner, 2024
 */
object EncoderTicks {
    /**
     * Convert encoder ticks to an angle.
     *
     * @param ticks The number of encoder ticks.
     * @param ticksPerRevolution The number of encoder ticks per revolution.
     * @param reduction The gear reduction (input speed / output speed).
     * @return The angle.
     */
    @JvmStatic
    fun toAngle(ticks: Int, ticksPerRevolution: Int, reduction: Double): Measure<Angle> {
        // Equation: ticks / ticksPerRevolution * 360 * reduction = angle (in degrees)
        return Degrees.of(ticks.toDouble() / ticksPerRevolution.toDouble() * 360.0).times(reduction)
    }

    /**
     * Convert an angle to encoder ticks.
     *
     * @param angle The angle.
     * @param ticksPerRevolution The number of encoder ticks per revolution.
     * @param reduction The gear reduction (input speed / output speed).
     * @return The number of encoder ticks.
     */
    @JvmStatic
    fun fromAngle(angle: Measure<Angle>, ticksPerRevolution: Int, reduction: Double): Int {
        // Equation: angle (in degrees) / 360 * ticksPerRevolution * reduction = ticks
        return ((angle.inUnit(Degrees) / 360.0 * ticksPerRevolution.toDouble()) * reduction).toInt()
    }

    /**
     * Convert encoder ticks to a distance.
     *
     * @param ticks The number of encoder ticks.
     * @param ticksPerRevolution The number of encoder ticks per revolution.
     * @param wheelDiameter The diameter of the wheel.
     * @param reduction The gear reduction (input speed / output speed).
     * @return The distance.
     */
    @JvmStatic
    fun toDistance(
        ticks: Int,
        ticksPerRevolution: Int,
        wheelDiameter: Measure<Distance>,
        reduction: Double
    ): Measure<Distance> {
        // Equation: circumference (2*pi*r_m) * (encoder ticks / ticksPerRevolution) = distance (in meters)
        return wheelDiameter.times(Math.PI).times(ticks.toDouble() / ticksPerRevolution.toDouble()).times(reduction)
    }

    /**
     * Convert a distance to encoder ticks.
     *
     * @param distance The distance.
     * @param ticksPerRevolution The number of encoder ticks per revolution.
     * @param wheelDiameter The diameter of the wheel.
     * @param reduction The gear reduction (input speed / output speed).
     * @return The number of encoder ticks.
     */
    @JvmStatic
    fun fromDistance(
        distance: Measure<Distance>,
        ticksPerRevolution: Int,
        wheelDiameter: Measure<Distance>,
        reduction: Double
    ): Int {
        // Equation: distance * ticksPerRevolution / circumference (2*pi*r_m) = ticks
        return ((distance.inUnit(Meters) * ticksPerRevolution.toDouble() / (wheelDiameter.inUnit(Meters) * Math.PI)) * reduction).toInt()
    }

    /**
     * Create a dynamic generator for encoder tick conversions based on a motor.
     * This will try to automatically determine the number of encoder ticks per revolution, if this is not provided,
     * use the other constructor.
     *
     * @param motor The motor.
     * @param reduction The gear reduction (input speed / output speed). If not provided, the reduction will be 1.
     * @param wheelDiameter The diameter of the wheel. If not provided, distance conversions will not be available.
     * @return The encoder tick generator.
     */
    @JvmStatic
    @JvmOverloads
    fun createGenerator(
        motor: DcMotorEx,
        reduction: Double = 1.0,
        wheelDiameter: Measure<Distance>? = null
    ): EncoderTickGenerator {
        return EncoderTickGenerator(motor, motor.motorType.ticksPerRev.toInt(), reduction, wheelDiameter)
    }

    /**
     * Create a dynamic generator for encoder tick conversions based on a motor.
     *
     * @param motor The motor.
     * @param ticksPerRevolution The number of encoder ticks per revolution.
     * @param reduction The gear reduction (input speed / output speed). If not provided, the reduction will be 1.
     * @param wheelDiameter The diameter of the wheel. If not provided, distance conversions will not be available.
     * @return The encoder tick generator.
     */
    @JvmStatic
    @JvmOverloads
    fun createGenerator(
        motor: DcMotorEx,
        ticksPerRevolution: Int,
        reduction: Double = 1.0,
        wheelDiameter: Measure<Distance>? = null
    ): EncoderTickGenerator {
        return EncoderTickGenerator(motor, ticksPerRevolution, reduction, wheelDiameter)
    }

    /**
     * Dynamic generator for encoder tick conversions based on a motor.
     */
    class EncoderTickGenerator(
        private val motor: DcMotorEx,
        private val ticksPerRevolution: Int,
        private val reduction: Double,
        private val wheelDiameter: Measure<Distance>?
    ) {
        /**
         * Get the current encoder ticks in an angle.
         */
        fun getAngle(): Measure<Angle> {
            return toAngle(motor.currentPosition, ticksPerRevolution, reduction)
        }

        /**
         * Get the angular velocity of the encoder.
         */
        fun getAngularVelocity(): Measure<Velocity<Angle>> {
            return toAngle(motor.velocity.toInt(), ticksPerRevolution, reduction).per(Second)
        }

        /**
         * Get the angular acceleration of the encoder.
         * Note that the motor attached to this generator *must* be an instance of [Motor], or an exception will be thrown.
         */
        fun getAngularAcceleration(): Measure<Velocity<Velocity<Angle>>> {
            if (motor !is Motor) {
                throw IllegalStateException("Motor attached to this generator is not a Motor instance. Acceleration information is not available.")
            }
            return toAngle(motor.acceleration.toInt(), ticksPerRevolution, reduction).per(Second).per(Second)
        }

        /**
         * Get the current encoder ticks in a distance.
         * @return the distance, else null if the wheel diameter is not provided.
         */
        fun getDistance(): Measure<Distance>? {
            return wheelDiameter?.let { toDistance(motor.currentPosition, ticksPerRevolution, it, reduction) }
        }

        /**
         * Get the current encoder velocity in a distance.
         * @return the distance, else null if the wheel diameter is not provided.
         */
        fun getVelocity(): Measure<Distance>? {
            return wheelDiameter?.let { toDistance(motor.velocity.toInt(), ticksPerRevolution, it, reduction) }
        }

        /**
         * Get the current encoder acceleration in a distance.
         * Note that the motor attached to this generator *must* be an instance of [Motor], or an exception will be thrown.
         */
        fun getAcceleration(): Measure<Velocity<Velocity<Distance>>>? {
            if (motor !is Motor) {
                throw IllegalStateException("Motor attached to this generator is not a Motor instance. Acceleration information is not available.")
            }
            return wheelDiameter?.let {
                toDistance(motor.acceleration.toInt(), ticksPerRevolution, it, reduction).per(
                    Second
                ).per(Second)
            }
        }

        /**
         * Get an angle from the provided encoder ticks.
         */
        fun angle(ticks: Int): Measure<Angle> {
            return toAngle(ticks, ticksPerRevolution, reduction)
        }

        /**
         * Get a distance from the provided encoder ticks.
         * @return the distance, else null if the wheel diameter is not provided.
         */
        fun distance(ticks: Int): Measure<Distance>? {
            return wheelDiameter?.let { toDistance(ticks, ticksPerRevolution, it, reduction) }
        }

        /**
         * Get encoder ticks from the provided angle.
         */
        fun ticks(angle: Measure<Angle>): Int {
            return fromAngle(angle, ticksPerRevolution, reduction)
        }

        /**
         * Get encoder ticks from the provided distance.
         * @return the number of encoder ticks, else null if the wheel diameter is not provided.
         */
        fun ticks(distance: Measure<Distance>): Int? {
            return wheelDiameter?.let { fromDistance(distance, ticksPerRevolution, it, reduction) }
        }
    }
}