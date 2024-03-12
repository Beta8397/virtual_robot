package org.murraybridgebunyips.bunyipslib

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Abstract class to use as parent to the class you will define to mirror a "saved configuration"
 * on the Robot Controller, and to define any robot related constants.
 * Supported for use in both BunyipsOpModes and any other normal SDK OpMode.
 * ```
 *     private final YourConfig config = new YourConfig();
 * ```
 * In your OpMode's init method, call the `init` method of your config class, passing in the OpMode.
 * ```
 *     config.init(this);
 * ```
 */
abstract class RobotConfig {
    protected lateinit var hardwareMap: HardwareMap

    /**
     * Here you will assign class instance variables to public HardwareDevices, and assign any runtime-related
     * settings and variables for use. This is called from the `init` method.
     */
    protected abstract fun onRuntime()

    /**
     * Uses the HardwareMap to fetch HardwareDevices and assign instances from `onRuntime`.
     * Should be called as the first line in your init cycle.
     * @param opMode the OpMode instance - usually the `this` object when at the root OpMode.
     */
    fun init(opMode: OpMode) {
        errors.clear()
        this.hardwareMap = opMode.hardwareMap
        onRuntime()
        if (opMode is BunyipsOpMode) {
            opMode.addTelemetry(
                "${this.javaClass.simpleName}: Configuration completed with ${errors.size} error(s).",
            )
        } else {
            opMode.telemetry.addData(
                "",
                "${this.javaClass.simpleName}: Configuration completed with ${errors.size} error(s).",
            )
        }
        if (errors.isNotEmpty()) {
            for (error in errors) {
                if (opMode is BunyipsOpMode) {
                    opMode.addRetainedTelemetry("! DEV_FAULT: $error")
                } else {
                    opMode.telemetry.addData("", "! DEV_FAULT: $error").setRetained(true)
                }
            }
        }
    }

    /**
     * Implicit OpMode config initialisation for use in BunyipsOpModes. This will not work in normal SDK OpModes.
     *
     * Uses the HardwareMap to fetch HardwareDevices and assign instances from `onRuntime`.
     * Should be called as the first line in your init cycle.
     *
     * @throws UnsupportedOperationException if not called from a BunyipsOpMode.
     * @see init(opMode: OpMode)
     */
    fun init() {
        try {
            // Access the singleton associated with a BunyipsOpMode, if we're not running one Kotlin
            // will throw a UninitializedPropertyAccessException, so we can tell the user off here.
            init(BunyipsOpMode.instance)
        } catch (e: UninitializedPropertyAccessException) {
            throw UnsupportedOperationException("Argument-less RobotConfig.init() method is only supported in a BunyipsOpMode. Use RobotConfig.init(opMode) instead.")
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
            hardwareDevice = hardwareMap.get(device, name) as HardwareDevice
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
