package org.murraybridgebunyips.bunyipslib

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.LightSensor
import com.qualcomm.robotcore.hardware.RobotCoreLynxUsbDevice
import com.qualcomm.robotcore.hardware.ServoController
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.Telemetry.Item
import org.murraybridgebunyips.bunyipslib.Text.formatString
import org.murraybridgebunyips.bunyipslib.Text.round
import org.murraybridgebunyips.bunyipslib.roadrunner.util.LynxModuleUtil
import org.murraybridgebunyips.deps.BuildConfig
import kotlin.math.roundToInt


/**
 * Base class for all OpModes that provides a number of useful methods and utilities for development.
 * Includes a lifecycle that is similar to an iterative lifecycle, but includes logging, error catching,
 * and abstraction to provide phased code execution.
 *
 * @author Lucas Bubner, 2023
 */
abstract class BunyipsOpMode : LinearOpMode() {
    lateinit var movingAverageTimer: MovingAverageTimer
        private set

    private var operationsCompleted = false
    private var operationsPaused = false
    private var safeHaltHardwareOnStop = false

    private var opModeStatus = "idle"
    private lateinit var overheadTelemetry: Item
    private var telemetryQueue = 0
    private val telemetryItems = mutableSetOf<Pair<ItemType, String>>()
    private var packet: TelemetryPacket? = TelemetryPacket()

    private enum class ItemType {
        TELEMETRY,
        RETAINED_TELEMETRY,
        LOG
    }

    companion object {
        /**
         * The instance of the current BunyipsOpMode. This is set automatically by the OpMode lifecycle.
         * This can be used instead of dependency injection to access the current OpMode, as it is a singleton.
         *
         * BunyipsComponent and Task internally use this to grant access to the current OpMode through
         * the `opMode` property. You must ensure all Tasks and BunyipsComponents are instantiated during runtime,
         * (such as during onInit()), otherwise this property will be null.
         *
         * If you choose to access the current OpMode through this property, you must ensure that the OpMode
         * is actively running, otherwise this property will be null and you will raise a whole suite of exceptions.
         */
        @JvmStatic
        lateinit var instance: BunyipsOpMode
            private set
    }

    /**
     * Runs upon the pressing of the INIT button on the Driver Station.
     * This is where you should initialise your hardware and other components.
     */
    protected abstract fun onInit()

    /**
     * Run code in a loop AFTER onInit() has completed, until
     * start is pressed on the Driver Station or true is returned to this method.
     * This method is called at least once.
     * If not implemented, the OpMode will continue on as normal and wait for start.
     */
    protected open fun onInitLoop(): Boolean {
        return true
    }

    /**
     * Allow code to execute once after all initialisation has finished.
     * Note: this method is always called even if initialisation is cut short by the Driver Station.
     */
    protected open fun onInitDone() {
    }

    /**
     * Perform one time operations after start is pressed.
     * Unlike onInitDone, this will only execute once play is hit and not when initialisation is done.
     */
    protected open fun onStart() {
    }

    /**
     * Code to run continuously after the START button is pressed on the Driver Station.
     * This method will be called on each hardware cycle, and is guaranteed to be called at least once.
     */
    protected abstract fun activeLoop()

    /**
     * Perform one time clean-up operations after the activeLoop() finishes from an uninterrupted
     * OpMode lifecycle. This method is called after a finish() or exit() call, but may not
     * be called if the OpMode is terminated by an unhandled exception. This method is useful for
     * ensuring the robot is in a safe state after the OpMode has finished.
     * @see onStop
     */
    protected open fun onFinish() {
    }

    /**
     * Perform one time clean-up operations after the OpMode finishes.
     * This method is called after the OpMode has been requested to stop, and will be the last method
     * called before the OpMode is terminated, and is *guaranteed* to be called. This method is useful
     * for releasing resources to prevent memory leaks, as motor controllers will be powered off
     * as the OpMode is ending.
     * This method is not exception protected!
     * @see onFinish
     */
    protected open fun onStop() {
    }

    /**
     * Main method overridden from LinearOpMode that handles the OpMode lifecycle.
     * @throws InterruptedException
     */
    @Throws(InterruptedException::class)
    final override fun runOpMode() {
        // BunyipsOpMode
        instance = this
        try {
            Dbg.log("=============== BunyipsLib BunyipsOpMode ${BuildConfig.GIT_COMMIT}-${BuildConfig.BUILD_TIME} uid:${BuildConfig.ID} ===============")
            LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)
            hardwareMap.getAll(LynxModule::class.java).forEach { module ->
                module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
            }
            // Separate log from telemetry on the DS with an empty line
            telemetry.log().add("")
            telemetry.log().add("bunyipslib ${BuildConfig.GIT_COMMIT}-${BuildConfig.BUILD_TIME}")
            telemetryItems.add(
                Pair(
                    ItemType.LOG,
                    "bunyipslib ${BuildConfig.GIT_COMMIT}-${BuildConfig.BUILD_TIME}"
                )
            )

            opModeStatus = "setup"
            Dbg.logd("BunyipsOpMode: setting up...")
            telemetry.log().displayOrder = Telemetry.Log.DisplayOrder.OLDEST_FIRST
            telemetry.captionValueSeparator = ""
            // Uncap the telemetry log limit to ensure we capture everything
            telemetry.log().capacity = 999999
            // Ring-buffer timing utility
            movingAverageTimer = MovingAverageTimer()
            // Assign overhead telemetry as it has been configured now
            overheadTelemetry =
                telemetry.addData("", "BOM: idle | T+0s | 0.000ms | (?) (?)\n")
                    .setRetained(true)
            pushTelemetry()

            opModeStatus = "static_init"
            Dbg.logd("BunyipsOpMode: firing onInit()...")
            // Store telemetry objects raised by onInit() by turning off auto-clear
            setTelemetryAutoClear(false)
            pushTelemetry()
            if (!gamepad1.atRest() || !gamepad2.atRest()) {
                log("WARNING: a gamepad was not zeroed during init. please ensure controllers zero out correctly.")
            }
            // Run user-defined setup
            try {
                onInit()
            } catch (e: Exception) {
                // Catch all exceptions, log them, and continue running the OpMode
                // All InterruptedExceptions are handled by the FTC SDK and are raised in ErrorUtil
                Exceptions.handle(e, ::log)
            }

            opModeStatus = "dynamic_init"
            pushTelemetry()
            Dbg.logd("BunyipsOpMode: starting onInitLoop()...")
            // Run user-defined dynamic initialisation
            do {
                try {
                    // Run until onInitLoop returns true or the OpMode is continued
                    if (onInitLoop()) break
                } catch (e: Exception) {
                    Exceptions.handle(e, ::log)
                }
                movingAverageTimer.update()
                pushTelemetry()
            } while (opModeInInit())

            opModeStatus = "finish_init"
            pushTelemetry()
            Dbg.logd("BunyipsOpMode: firing onInitDone()...")
            // Run user-defined final initialisation
            try {
                onInitDone()
            } catch (e: Exception) {
                Exceptions.handle(e, ::log)
            }

            // Ready to go.
            opModeStatus = "ready"
            movingAverageTimer.update()
            Dbg.logd("BunyipsOpMode: init cycle completed in ${movingAverageTimer.elapsedTime() / 1000.0} secs")
            // DS only telemetry
            telemetry.addData("", "BunyipsOpMode: INIT COMPLETE -- PLAY WHEN READY.")
            Dbg.logd("BunyipsOpMode: ready.")
            movingAverageTimer.reset()

            // Wait for start
            do {
                pushTelemetry()
            } while (!isStarted)

            // Play button has been pressed
            opModeStatus = "starting"
            setTelemetryAutoClear(true)
            clearTelemetry()
            pushTelemetry()
            movingAverageTimer.reset()
            Dbg.logd("BunyipsOpMode: firing onStart()...")
            try {
                // Run user-defined start operations
                onStart()
            } catch (e: Exception) {
                Exceptions.handle(e, ::log)
            }

            opModeStatus = "running"
            Dbg.logd("BunyipsOpMode: starting activeLoop()...")
            do {
                if (operationsPaused) {
                    // If the OpMode is paused, skip the loop and wait for the next hardware cycle
                    opModeStatus = "halted"
                    movingAverageTimer.update()
                    pushTelemetry()
                    continue
                }
                try {
                    // Run user-defined active loop
                    activeLoop()
                } catch (e: Exception) {
                    Exceptions.handle(e, ::log)
                }
                // Update telemetry and timers
                movingAverageTimer.update()
                pushTelemetry()
            } while (opModeIsActive() && !operationsCompleted)

            opModeStatus = "finished"
            try {
                onFinish()
            } catch (e: Exception) {
                Exceptions.handle(e, ::log)
            }
            // overheadTelemetry will no longer update, will remain frozen on last value
            movingAverageTimer.update()
            pushTelemetry()
            Dbg.logd("BunyipsOpMode: all tasks finished.")
            // Wait for user to hit stop or for the OpMode to be terminated
            // We will continue running the OpMode as there might be some other user threads
            // under Threads that can still run, so we will wait indefinitely and simulate
            // what the FTC SDK does when the OpMode is stopped if they wish.
            // This has been made optional as users may want to hold a position or keep a motor
            // running after the OpMode has finished
            while (opModeIsActive()) {
                if (safeHaltHardwareOnStop)
                    safeHaltHardware()
            }
        } catch (t: Throwable) {
            Dbg.error("BunyipsOpMode: unhandled throwable! <${t.message}>")
            Dbg.sendStacktrace(t)
            // As this error occurred somewhere not inside user code, we should not swallow it.
            // There is also a chance this error is an instance of Error, in which case we should
            // exit immediately as something has gone very wrong
            throw t
        } finally {
            Dbg.logd("BunyipsOpMode: opmode stop requested. cleaning up...")
            // Ensure all user threads have been told to stop
            Threads.stopAll()
            onStop()
            // Telemetry may be not in a nice state, so we will call our stateful functions
            // such as thread stops and cleanup in onStop() first before updating the status
            opModeStatus = "terminating"
            Dbg.logd("BunyipsOpMode: active cycle completed in ${movingAverageTimer.elapsedTime() / 1000.0} secs")
            pushTelemetry()
            Dbg.logd("BunyipsOpMode: exiting...")
        }
    }

    /**
     * Update and push queued telemetry to the Driver Station and FtcDashboard.
     */
    fun pushTelemetry() {
        // Update main DS telemetry
        telemetry.update()

        // Requeue new overhead status message
        val loopTime = round(movingAverageTimer.movingAverage(), 2)
        val loopsSec = if (!movingAverageTimer.loopsSec().isNaN())
            round(movingAverageTimer.loopsSec(), 1)
        else 0.0

        val overheadStatus =
            "$opModeStatus | T+${(movingAverageTimer.elapsedTime() / 1000).roundToInt()}s | ${
                if (loopTime <= 0.0) {
                    if (loopsSec > 0) "$loopsSec l/s" else "0.00ms"
                } else {
                    "${loopTime}ms"
                }
            } | ${Controller.movementString(gamepad1)} ${Controller.movementString(gamepad2)}\n"

        overheadTelemetry.setValue("BOM: $overheadStatus")

        // FtcDashboard
        if (packet == null) {
            packet = TelemetryPacket()
        }

        packet?.let {
            it.put("BOM", overheadStatus + "\n")

            // Copy with toList() to avoid ConcurrentModificationExceptions
            telemetryItems.toList().forEachIndexed { index, pair ->
                val (type, value) = pair
                when (type) {
                    ItemType.TELEMETRY -> it.put("DS$index", value)
                    ItemType.RETAINED_TELEMETRY -> it.put("RT$index", value)
                    ItemType.LOG -> {
                        if (index == 0) {
                            // BunyipsLib info, this is an always log and will always
                            // be the first log in the list as it is added at the start
                            // of the init cycle
                            it.put("INFO", value)
                            return@forEachIndexed
                        }
                        it.put("LOG$index", value)
                    }
                }
            }

            FtcDashboard.getInstance().sendTelemetryPacket(it)

            // Invalidate this packet
            packet = null
        }

        if (telemetry.isAutoClear) {
            telemetryQueue = 0
            clearTelemetryObjects()
        }
    }

    /**
     * Add any additional telemetry to the FtcDashboard telemetry packet.
     */
    fun addDashboardTelemetry(key: String, value: Any?) {
        if (packet == null) {
            packet = TelemetryPacket()
        }
        packet!!.put(key, value.toString())
    }

    /**
     * Add any field overlay data to the FtcDashboard telemetry packet.
     */
    fun dashboardFieldOverlay(): Canvas {
        // This will allow us to attach any field overlay data to the packet
        // as we will only reassign if the packet is null
        if (packet == null) {
            packet = TelemetryPacket()
        }
        return packet!!.fieldOverlay()
    }

    /**
     * Ensure an exception is not thrown due to the telemetry queue being bigger than 255 objects.
     */
    private fun flushTelemetryQueue() {
        telemetryQueue++
        if (telemetryQueue >= 255) {
            // We have to flush out telemetry as the queue is too big
            pushTelemetry()
            if (telemetry.isAutoClear) {
                // Flush successful
                Dbg.logd("Telemetry queue exceeded 255 messages, auto-pushing to flush...")
            }
        }
    }

    /**
     * Create a new telemetry object and add it to the management queue.
     */
    private fun createTelemetryItem(value: String, retained: Boolean): Item {
        val item = telemetry.addData(value, "")
            .setRetained(retained)
        if (value.isNotBlank()) {
            telemetryItems.add(
                Pair(
                    if (retained) ItemType.RETAINED_TELEMETRY else ItemType.TELEMETRY,
                    value
                )
            )
        }
        return item
    }

    /**
     * Clear all telemetry objects from the management queue just like a telemetry.clear() call.
     */
    private fun clearTelemetryObjects() {
        val tmp = telemetryItems.toTypedArray()
        for (item in tmp) {
            if (item.first == ItemType.RETAINED_TELEMETRY)
                continue
            telemetryItems.remove(item)
        }
    }

    /**
     * Add data to the telemetry object, with integrated formatting.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     * @return The telemetry item added to the Driver Station, null if the send failed from overflow
     */
    fun addTelemetry(format: Any, vararg args: Any?): Item? {
        flushTelemetryQueue()
        if (telemetryQueue >= 255 && !telemetry.isAutoClear) {
            // Auto flush will fail as clearing is not permitted
            // We will send telemetry to the debugger instead as a fallback
            Dbg.log("Telemetry overflow: $format")
            return null
        }
        return createTelemetryItem(formatString(format.toString(), *args), false)
    }


    /**
     * Add a data to the telemetry object, with integrated formatting.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     * @return The telemetry item added to the Driver Station
     */
    fun addRetainedTelemetry(format: Any, vararg args: Any?): Item {
        flushTelemetryQueue()
        // Retained objects will never be cleared, so we can just add them immediately, as usually
        // retained messages should be important and not be discarded
        return createTelemetryItem(formatString(format.toString(), *args), true)
    }

    /**
     * Remove retained entries from the telemetry object.
     * @param items The items to remove from the telemetry object
     */
    fun removeRetainedTelemetry(vararg items: Item) {
        for (item in items) {
            // We will be able to remove the item from the DS, but objects on FtcDashboard cannot
            // be removed as we no longer know the index of the object or the contents of the item.
            // This means all RT objects on the dashboard are permanent, and will respect their
            // last updated value. This is not a problem as retained objects are usually important
            // and can serve as a debugging tool.
            val res = telemetry.removeItem(item)
            if (!res) {
                Dbg.logd("Could not find telemetry item to remove: $item")
                return
            }
        }
    }

    fun removeRetainedTelemetry(items: List<Item>) {
        removeRetainedTelemetry(*items.toTypedArray())
    }

    /**
     * Log a message to the telemetry log, with integrated formatting.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    fun log(format: Any, vararg args: Any?) {
        val fstring = formatString(format.toString(), *args)
        telemetry.log().add(fstring)
        telemetryItems.add(Pair(ItemType.LOG, fstring))
    }

    /**
     * Log a message to the telemetry log, with integrated formatting.
     * @param obj Class where this log was called (name will be prepended to message)
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    fun log(obj: Class<*>, format: Any, vararg args: Any?) {
        val msg = "[${obj.simpleName}] ${formatString(format.toString(), *args)}"
        telemetry.log().add(msg)
        telemetryItems.add(Pair(ItemType.LOG, msg))
    }

    /**
     * Log a message into the telemetry log
     * @param stck StackTraceElement with information about where this log was called (see Text.getCallingUserCodeFunction())
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    fun log(stck: StackTraceElement, format: Any, vararg args: Any?) {
        val msg = "[${stck}] ${formatString(format.toString(), *args)}"
        telemetry.log().add(msg)
        telemetryItems.add(Pair(ItemType.LOG, msg))
    }

    /**
     * Reset telemetry data, including retention
     */
    fun resetTelemetry() {
        telemetryQueue = 0
        telemetry.clearAll()
        telemetryItems.clear()
        FtcDashboard.getInstance().clearTelemetry()
        // Must reassign the overhead telemetry item
        overheadTelemetry =
            telemetry.addData("", "BOM: unknown | T+?s | ?ms | (?) (?)")
                .setRetained(true)
    }

    /**
     * Clear telemetry on screen, not including retention
     */
    fun clearTelemetry() {
        telemetryQueue = 0
        clearTelemetryObjects()
        telemetry.clear()
    }

    /**
     * Set telemetry auto clear status
     */
    fun setTelemetryAutoClear(autoClear: Boolean) {
        telemetry.isAutoClear = autoClear
    }

    /**
     * Get auto-clear status of telemetry
     */
    fun getTelemetryAutoClear(): Boolean {
        return telemetry.isAutoClear
    }

    /**
     * Call to manually finish the OpMode.
     * This is a dangerous method, as the OpMode will no longer be able to run any main thread code.
     * This method should be called when the OpMode is finished and no longer needs to run, and will
     * will put the OpMode in a state where it will not run any more code (including timers & telemetry).
     * @param safeHaltHardwareOnStop If true (default), all motors/devices will be actively told to stop for the remainder of the OpMode.
     */
    fun finish(safeHaltHardwareOnStop: Boolean = true) {
        if (operationsCompleted) {
            return
        }
        this.safeHaltHardwareOnStop = safeHaltHardwareOnStop
        operationsCompleted = true
        Dbg.logd("BunyipsOpMode: activeLoop() terminated by finish().")
        createTelemetryItem(
            "BunyipsOpMode: Robot ${if (safeHaltHardwareOnStop) "is stopped" else "tasks finished"}. All operations completed.",
            true
        )
        pushTelemetry()
    }

    /**
     * Call to manually finish the OpMode.
     * This is a dangerous method, as the OpMode will no longer be able to run any main thread code.
     * This method should be called when the OpMode is finished and no longer needs to run, and will
     * will put the OpMode in a state where it will not run any more code (including timers & telemetry).
     * This method will also actively tell all motors/devices to stop for the remainder of the OpMode.
     */
    fun finish() {
        finish(true)
    }

    /**
     * Call to command all motors and sensors on the robot to stop.
     * This method is continuously called when no OpMode is running, and allows you to do the same while still
     * in an OpMode (called internally after `finish(true)`).
     * This method will also power down all LynxModules and disable all servos.
     */
    fun safeHaltHardware() {
        // Set all motor powers to zero, the implementation here will also stop any CRServos
        for (motor in hardwareMap.getAll(DcMotorSimple::class.java)) {
            // Avoid enabling servos if they are already zero power
            if (motor.power != 0.0) motor.power = 0.0
        }
        // Shut down all LynxModules
        for (device in hardwareMap.getAll(RobotCoreLynxUsbDevice::class.java)) {
            device.failSafe()
        }
        // Power down the servos
        for (servoController in hardwareMap.getAll(ServoController::class.java)) {
            if (servoController.manufacturer == HardwareDevice.Manufacturer.Lynx) {
                servoController.pwmDisable()
            }
        }
        // Set motors to safe state
        for (dcMotor in hardwareMap.getAll(DcMotor::class.java)) {
            if (dcMotor.manufacturer == HardwareDevice.Manufacturer.Lynx) {
                dcMotor.power = 0.0
                dcMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
        }
        // Turn off light sensors
        for (light in hardwareMap.getAll(LightSensor::class.java)) {
            light.enableLed(false)
        }
    }

    /**
     * Call to temporarily halt all activeLoop-related updates from running.
     * Note this will pause the entire activeLoop, but continue to update timers and telemetry. These events
     * must be handled manually if needed, which include any conditional calls to resume().
     */
    fun halt() {
        if (operationsPaused) {
            return
        }
        operationsPaused = true
        Dbg.logd("BunyipsOpMode: activeLoop() halted.")
    }

    /**
     * Call to resume the activeLoop after a halt() call.
     */
    fun resume() {
        if (!operationsPaused) {
            return
        }
        operationsPaused = false
        opModeStatus = "running"
        Dbg.logd("BunyipsOpMode: activeLoop() resumed.")
    }

    /**
     * Dangerous method: call to shut down the OpMode as soon as possible.
     * This will run any BunyipsOpMode cleanup code.
     */
    fun exit() {
        Dbg.logd("BunyipsOpMode: exiting opmode...")
        finish()
        requestOpModeStop()
    }

    /**
     * Dangerous method: call to IMMEDIATELY terminate the OpMode.
     * This will not run any cleanup code, and should only be used in emergencies.
     */
    fun emergencyStop() {
        Dbg.logd("BunyipsOpMode: emergency stop requested.")
        terminateOpModeNow()
    }
}
