package org.murraybridgebunyips.bunyipslib

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.Objects

/**
 * Abstract class to use as parent to the class you will define to mirror a "saved configuration" on the Robot Controller
 * ```
 *     private final YourConfig config = new YourConfig();
 *
 *     @Override
 *     protected void onInit() {
 *         config.init();
 *     }
 * ```
 */
abstract class RobotConfig {
    protected var hardwareMap: HardwareMap? = null

    /**
     * Assign class instance variables to public HardwareDevices.
     */
    protected abstract fun configureHardware()

    /**
     * Use HardwareMap to fetch HardwareDevices and assign instances.
     * Should be called as the first line in your init cycle.
     * @param opMode the OpMode instance - usually the `this` object when at the root OpMode.
     */
    fun init(opMode: OpMode) {
        if (opMode is BunyipsOpMode) {
            init()
            return
        }
        errors.clear()
        this.hardwareMap = opMode.hardwareMap
        Objects.requireNonNull(
            this.hardwareMap,
            "HardwareMap was null in ${this.javaClass.simpleName}!"
        )
        configureHardware()
        opMode.telemetry.addData(
            "",
            "${this.javaClass.simpleName}: Configuration completed with ${errors.size} error(s).",
        )
        if (errors.isNotEmpty()) {
            for (error in errors) {
                opMode.telemetry.addData("", "! DEV_FAULT: $error").setRetained(true)
            }
        }
    }

    /**
     * Use HardwareMap to fetch HardwareDevices and assign instances.
     * Should be called as the first line in onInit();
     *
     * Argument-less init() cannot be used with a non-BunyipsOpMode instance (will throw an UninitializedPropertyAccessException)
     */
    fun init() {
        val opMode = BunyipsOpMode.instance
        errors.clear()
        this.hardwareMap = opMode.hardwareMap
        Objects.requireNonNull(
            this.hardwareMap,
            "HardwareMap was null in ${this.javaClass.simpleName}!"
        )
        configureHardware()
        opMode.addTelemetry(
            "${this.javaClass.simpleName}: Configuration completed with ${errors.size} error(s).",
        )
        if (errors.isNotEmpty()) {
            for (error in errors) {
                opMode.addRetainedTelemetry("! DEV_FAULT: $error")
            }
        }
    }

    /**
     * Convenience method for reading the device from the hardwareMap without having to check for exceptions.
     * Uses class initialistion instead of hardwareMap initialistion to widen the range of devices, supporting
     * custom classes for dead wheels, etc.
     *
     * Every hardware error with this method be saved to a static array, which can be accessed during the
     * lifetime of the opMode.
     *
     * @param name   name of device saved in the configuration file
     * @param device the class of the item to configure, in final abstraction extending HardwareDevice
     */
    fun getHardware(name: String, device: Class<*>?): HardwareDevice? {
        var hardwareDevice: HardwareDevice? = null
        try {
            hardwareDevice = hardwareMap?.get(device, name) as HardwareDevice
        } catch (e: Throwable) {
            errors.add(name)
            e.localizedMessage?.let { Dbg.warn(it) }
        }
        return hardwareDevice
    }

    // Global storage objects
    companion object {
        /**
         * Static array of hardware errors stored via hardware name.
         */
        @JvmStatic
        val errors = ArrayList<String>()

        /**
         * Static Pose2d to store the robot's last known position after an OpMode has ended.
         */
        @JvmStatic
        var lastKnownPosition: Pose2d? = null
    }
}
