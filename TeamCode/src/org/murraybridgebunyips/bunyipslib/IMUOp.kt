package org.murraybridgebunyips.bunyipslib

import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation

/**
 * IMUOperation custom common class for internal IMUs
 *
 * This class is used to abstract the IMU reading and provide a more human-friendly reading, serving
 * as a wrapper for the internal IMU class. RoadRunner uses the direct IMU class, so this class is
 * only for use in other situations such as reading the IMU for other tasks.
 *
 * This code has been updated to use the new SDK v8.1 specification on 03/06/2023.
 *
 * @author Lucas Bubner, 2022
 */
class IMUOp(private val imu: IMU) : BunyipsSubsystem() {
    init {
        assertParamsNotNull(imu)
    }

    /**
     * Get the current IMU reading from the internal IMU.
     */
    @Volatile
    var currentAngles: Orientation? = null

    /**
     * Get the current IMU velocity reading from the internal IMU.
     */
    @Volatile
    var currentVelocity: AngularVelocity? = null
    private var previousHeading = 0.0

    /**
     * Get the current PrecisionDrive IMU capture reading from the internal IMU.
     */
    var capture: Double? = null
        private set

    /**
     * Offset the IMU reading for field-centric navigation
     * Must be an polar angle: v E [0, 360], will be converted to a signed [-180, 180] angle
     */
    var offset = 0.0
        set(value) {
            if (value > 180) {
                field = value - 360
                return
            }
            field = value
        }

    /**
     * Get the current heading reading from the internal IMU, with support for absolute degree readings
     * Instead of using signed [-180, 180] readings, this will return a degree within -inf to +inf
     * @return Z degree value of Orientation axes in human-friendly reading range [-inf, inf]
     */
    var heading: Double = 0.0
        get() {
            val currentHeading = currentAngles?.thirdAngle?.toDouble()?.plus(offset) ?: return field
            var delta = currentHeading - previousHeading

            // Detects if there is a sudden 180 turn which means we have turned more than the 180
            // degree threshold. Adds 360 to additively inverse the value and give us a proper delta
            if (delta < -180) {
                delta += 360.0
            } else if (delta >= 180) {
                delta -= 360.0
            }

            field += delta
            previousHeading = currentHeading

            // Switched to using [-inf, inf] bounding instead of [0, 360] due to tasks detecting
            // positions based on greater than measurements, which would not work with 0-360
//            if (field >= 360.0) {
//                field -= 360.0
//            } else if (field < 0.0) {
//                field += 360.0
//            }

            return field
        }
        private set

    /**
     * Get the current signed [-180, 180] yaw reading from the internal IMU
     */
    val rawHeading: Double
        get() = currentAngles?.thirdAngle?.toDouble()?.plus(offset) ?: 0.0

    /**
     * Get Z rotation rate (heading rate) from the internal IMU
     */
    var turnRate: Double = 0.0
        get() {
            field = currentVelocity?.zRotationRate?.toDouble() ?: field
            return field
        }

    /**
     * Reset all heading measurements, offsets, and internal measurements to default 0.0
     */
    fun resetHeading(): IMUOp {
        heading = 0.0
        offset = 0.0
        imu.resetYaw()
        return this
    }

    /**
     * Update the latest state in the IMU to current data
     */
    override fun periodic() {
        this.currentAngles = imu.getRobotOrientation(
            AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES
        )
        this.currentVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES)
    }

    /**
     * Get the current roll reading from the internal IMU
     * @return Y value of Orientation axes, returns 0.0 as default value
     */
    var roll: Double = 0.0
        get() {
            field = this.currentAngles?.secondAngle?.toDouble() ?: field
            return field
        }

    /**
     * Get Y rotation rate (roll rate) from the internal IMU
     */
    var rollRate: Double = 0.0
        get() {
            field = this.currentVelocity?.yRotationRate?.toDouble() ?: field
            return field
        }

    /**
     * Get the current pitch reading from the internal IMU
     * @return X value of Orientation axes, returns 0.0 as default value
     */
    var pitch: Double = 0.0
        get() {
            field = this.currentAngles?.firstAngle?.toDouble() ?: field
            return field
        }

    /**
     * Get X rotation rate (pitch rate) from the internal IMU
     */
    var pitchRate: Double = 0.0
        get() {
            field = this.currentVelocity?.xRotationRate?.toDouble() ?: field
            return field
        }

    /**
     * Start PrecisionDrive IMU alignment algorithm and capture the original angle
     */
    fun startCapture(): IMUOp {
        this.update()
        this.capture = this.heading
        return this
    }

    /**
     * Stop and reset PrecisionDrive IMU alignment algorithm
     */
    fun resetCapture(): IMUOp {
        this.capture = null
        return this
    }

    /**
     * Query motor alignment speed for ROTATIONAL speed through PrecisionDrive
     * @param originalSpeed supply the intended speed for the R SPEED value
     * @param tolerance supply the tolerance in degrees for the IMU to be within to stop adjusting
     * @return queried speed based on parameters given, returns the unaltered speed if PrecisionDrive is not online
     */
    fun getRPrecisionSpeed(originalSpeed: Double, tolerance: Double): Double {
        // If we're not capturing, return the original speed
        if (this.capture == null) return originalSpeed

        this.update()
        val current = this.heading

        // If we're at the minimum tolerance, increase turn rate
        if (current < this.capture!! - tolerance) return originalSpeed - 0.1

        // If we're at maximum tolerance, decrease turn rate
        return if (current > this.capture!! + tolerance) originalSpeed + 0.1 else originalSpeed
    }
}