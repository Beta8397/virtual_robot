package org.murraybridgebunyips.bunyipslib

import com.acmerobotics.dashboard.canvas.Canvas
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.LightSensor
import com.qualcomm.robotcore.hardware.RobotCoreLynxUsbDevice
import com.qualcomm.robotcore.hardware.ServoController
import com.qualcomm.robotcore.util.ThreadPool
import org.firstinspires.ftc.robotcore.external.Telemetry.Item
import org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds
import org.murraybridgebunyips.bunyipslib.roadrunner.util.LynxModuleUtil
import org.murraybridgebunyips.bunyipslib.tasks.bases.RobotTask
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task
import org.murraybridgebunyips.deps.BuildConfig
import java.util.concurrent.ExecutorService


/**
 * [BunyipsOpMode]
 *
 * The gateway to the BunyipsLib framework, BunyipsOpMode is a wrapper around the FTC SDK's LinearOpMode.
 *
 * This class provides a structured way to manage the lifecycle of an OpMode, and provides a number of
 * utility functions to make development easier. This class is designed to be extended by the user to
 * create their own OpModes, and has native support for linked Driver Station & FtcDashboard telemetry ([DualTelemetry]),
 * native custom gamepads ([Controller]), timing utilities ([timer]), exception handling ([Exceptions]),
 * and more through the BunyipsLib suite of tools ([BunyipsSubsystem], [Scheduler], etc.)
 *
 * @see CommandBasedBunyipsOpMode
 * @see AutonomousBunyipsOpMode
 * @author Lucas Bubner, 2024
 */
abstract class BunyipsOpMode : BOMInternal() {
    /**
     * The moving average timer for the OpMode, which is used to calculate time
     * between hardware cycles. This is useful for debugging, performance monitoring, and calculating various
     * time-based values (deltaTime, loopCount, elapsedTime, etc.)
     *
     * This timer is automatically active during the `dynamic_init` and `running` phases of the OpMode, and is reset in between.
     * If you wish to get the *total* time since this OpMode was started, you can call the built-in [getRuntime] method.
     * @see MovingAverageTimer
     */
    lateinit var timer: MovingAverageTimer

    /**
     * BunyipsLib Gamepad 1: Driver
     * @see Controller
     */
    lateinit var gamepad1: Controller

    /**
     * BunyipsLib Gamepad 2: Operator
     * @see Controller
     */
    lateinit var gamepad2: Controller

    /**
     * BunyipsLib Driver Station & FtcDashboard Telemetry
     * @see DualTelemetry
     */
    lateinit var telemetry: DualTelemetry

    private var operationsCompleted = false
    private var operationsPaused = false
    private var safeHaltHardwareOnStop = false

    private var gamepadExecutor: ExecutorService? = null
    private var initTask: RobotTask? = null

    companion object {
        private var _instance: BunyipsOpMode? = null

        /**
         * The instance of the current [BunyipsOpMode]. This is set automatically by the [BunyipsOpMode] lifecycle.
         * This can be used instead of dependency injection to access the current OpMode, as it is a singleton.
         *
         * `BunyipsComponent` and `Task` internally use this to grant access to the current OpMode through
         * the `opMode` property. As such, you must ensure all `Task`s and `BunyipsComponent`s are instantiated during runtime,
         * (such as during [onInit]), otherwise this property will be null.
         *
         * If you choose to access the current OpMode through this property, you must ensure that the OpMode
         * is actively running, otherwise this property will be null and you will raise an exception.
         *
         * @throws UninitializedPropertyAccessException If a [BunyipsOpMode] is not running, this exception will be raised.
         * @return The instance of the current [BunyipsOpMode].
         */
        @JvmStatic
        val instance: BunyipsOpMode
            // If Kotlin throws an UninitializedPropertyAccessException, it will crash the DS and require a full
            // restart, so we will handle this exception ourselves and supply a more informative message.
            get() = _instance
                ?: throw UninitializedPropertyAccessException("Attempted to access a BunyipsOpMode that is not running, this may be due to a task or subsystem being instantiated in the constructor or member fields. All subsystems and tasks must be instantiated during runtime, such as in onInit().")

        /**
         * Whether a [BunyipsOpMode] is currently running. This is useful for checking if the OpMode singleton can be accessed
         * without raising an exception due to the field being null.
         * @return Whether a [BunyipsOpMode] is currently running.
         */
        @JvmStatic
        val isRunning: Boolean
            get() = _instance != null
    }

    /**
     * Runs upon the pressing of the `INIT` button on the Driver Station.
     * This is where you should initialise your hardware and other components.
     */
    protected abstract fun onInit()

    /**
     * Run code in a loop AFTER [onInit] has completed, until
     * start is pressed on the Driver Station and the init-task ([setInitTask]) is done.
     * This method is called at least once.
     * If not implemented and no init-task is defined, the OpMode will continue on as normal and wait for start.
     */
    protected open fun onInitLoop(): Boolean {
        return true
    }

    /**
     * Allow code to execute once after all initialisation has finished.
     * Note: this method is always called even if initialisation is cut short by the Driver Station.
     */
    protected open fun onInitDone() {
        // no-op
    }

    /**
     * Perform one time operations after start is pressed.
     * Unlike [onInitDone], this will only execute once play is hit and not when initialisation is done.
     */
    protected open fun onStart() {
        // no-op
    }

    /**
     * Code to run continuously after the `START` button is pressed on the Driver Station.
     * This method will be called on each hardware cycle, but may not be called if the OpMode is stopped before starting.
     */
    protected abstract fun activeLoop()

    /**
     * Perform one time clean-up operations after the [activeLoop] finishes all intentions gracefully.
     * This method is called after a [finish] or [exit] call, but may not
     * be called if the OpMode is terminated by an *unhandled fatal* exception. This method is useful for
     * ensuring the robot is in a safe state after the OpMode has finished. In an exception-less OpMode,
     * this method will be called before [onStop].
     * @see onStop
     */
    protected open fun onFinish() {
        // no-op
    }

    /**
     * Perform one time clean-up operations as the OpMode is stopping.
     * This method is called after the OpMode has been requested to stop, and will be the last method
     * called before the OpMode is terminated, and is *guaranteed* to be called. This method is useful
     * for releasing resources to prevent memory leaks, as motor controllers will be powered off
     * as the OpMode is ending.
     * @see onFinish
     */
    protected open fun onStop() {
        // no-op
    }

    /**
     * This method is the entry point for the BunyipsLib framework, and is called by the FTC SDK.
     * @throws InterruptedException If the OpMode is interrupted by the FTC SDK, this exception will be raised.
     */
    @Throws(InterruptedException::class)
    final override fun runBunyipsOpMode() {
        // BunyipsOpMode
        _instance = this
        try {
            resetFields()
            Dbg.log("=============== BunyipsLib v${BuildConfig.SEMVER} BunyipsOpMode ${BuildConfig.GIT_COMMIT}-${BuildConfig.BUILD_TIME} uid:${BuildConfig.ID} ===============")
            LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)
            hardwareMap.getAll(LynxModule::class.java).forEach { module ->
                module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
            }

            Dbg.logd("BunyipsOpMode: setting up...")
            // Ring-buffer timing utility
            timer = MovingAverageTimer()
            // Telemetry
            telemetry = DualTelemetry(
                this,
                timer,
                "<b>${javaClass.simpleName}</b>",
                "<small><font color='#e5ffde'>bunyipslib</font> <font color='gray'>v${BuildConfig.SEMVER}-${BuildConfig.GIT_COMMIT}-${BuildConfig.BUILD_TIME}</font></small>"
            )
            // Controller setup and monitoring threads
            gamepad1 = Controller(sdkGamepad1)
            gamepad2 = Controller(sdkGamepad2)
            gamepadExecutor = ThreadPool.newFixedThreadPool(2, "BunyipsLib BunyipsOpMode Gamepad1+2")
            gamepadExecutor?.submit {
                while (!Thread.currentThread().isInterrupted)
                    gamepad1.update()
            }
            gamepadExecutor?.submit {
                while (!Thread.currentThread().isInterrupted)
                    gamepad2.update()
            }
            pushTelemetry()

            telemetry.opModeStatus = "<b><font color='yellow'></b>static_init</font>"
            Dbg.logd("BunyipsOpMode: firing onInit()...")
            // Store telemetry objects raised by onInit() by turning off auto-clear
            telemetry.isAutoClear = false
            pushTelemetry()
            if (!gamepad1.atRest() || !gamepad2.atRest()) {
                log("<b><font color='yellow'>WARNING!</font></b> a gamepad was not zeroed during init. please ensure controllers zero out correctly.")
            }
            // Run user-defined setup
            try {
                onInit()
            } catch (e: Exception) {
                // Catch all exceptions, log them, and continue running the OpMode
                // All InterruptedExceptions are handled by the FTC SDK and are raised in the Exceptions handler
                telemetry.overrideStatus = "<font color='red'><b>error</b></font>"
                Exceptions.handle(e, ::log)
            }

            telemetry.opModeStatus = "<font color='aqua'>dynamic_init</font>"
            pushTelemetry()
            Dbg.logd("BunyipsOpMode: starting onInitLoop()...")
            if (initTask != null) {
                Dbg.logd(
                    "BunyipsOpMode: running initTask -> % ...",
                    if (initTask is Task) (initTask as Task).toVerboseString() else initTask
                )
                log("<font color='gray'>running init-task:</font> %", initTask)
            }
            // Run user-defined dynamic initialisation
            do {
                try {
                    // Run the user's init task, if it isn't null
                    initTask?.run()
                    // Run until onInitLoop returns true, and the initTask is done, or the OpMode is continued
                    if (onInitLoop() && (initTask == null || initTask?.pollFinished() == true)) break
                } catch (e: Exception) {
                    telemetry.overrideStatus = "<font color='red'><b>error</b></font>"
                    Exceptions.handle(e, ::log)
                }
                timer.update()
                pushTelemetry()
            } while (opModeInInit() && !operationsCompleted)

            telemetry.opModeStatus = "<font color='yellow'>finish_init</font>"
            // We can only finish Task objects, as the RobotTask interface does not have a finish method
            // Most tasks will inherit from Task, so this should be safe, but this is to ensure maximum compatibility
            if (initTask is Task && initTask?.isFinished() == false) {
                Dbg.log("BunyipsOpMode: initTask did not finish in time, early finishing -> % ...", initTask)
                log("<font color='gray'>init-task interrupted by start request.</font>")
                (initTask as Task).finishNow()
            } else if (initTask?.isFinished() == false) {
                // If we can't finish the task, log a warning and continue
                Dbg.warn("BunyipsOpMode: initTask did not finish during the init-phase!", initTask)
                log("<b><font color='yellow'>WARNING!</font></b> <font color='gray'>init-task did not finish.</font>")
            } else if (initTask != null) {
                Dbg.logd("BunyipsOpMode: initTask finished -> % ...", initTask)
                log("<font color='gray'>init-task finished.</font>")
            }
            pushTelemetry()
            Dbg.logd("BunyipsOpMode: firing onInitDone()...")
            // Run user-defined final initialisation
            try {
                onInitDone()
            } catch (e: Exception) {
                telemetry.overrideStatus = "<font color='red'><b>error</b></font>"
                Exceptions.handle(e, ::log)
            }

            // Ready to go.
            telemetry.opModeStatus = "<font color='green'>ready</font>"
            timer.update()
            Dbg.logd("BunyipsOpMode: init cycle completed in ${timer.elapsedTime().inUnit(Seconds)} secs")
            // DS only telemetry
            telemetry.addDS("<b>Init <font color='green'>complete</font>. Press play to start.</b>")
            Dbg.logd("BunyipsOpMode: ready.")
            timer.reset()

            // Wait for start
            do {
                pushTelemetry()
                // Save some CPU cycles
                sleep(200)
            } while (!isStarted)

            // Play button has been pressed
            telemetry.opModeStatus = "<font color='yellow'>starting</font>"
            telemetry.isAutoClear = true
            clearTelemetry()
            pushTelemetry()
            timer.reset()
            Dbg.logd("BunyipsOpMode: firing onStart()...")
            try {
                // Run user-defined start operations
                onStart()
                telemetry.overrideStatus = null
            } catch (e: Exception) {
                telemetry.overrideStatus = "<font color='red'><b>error</b></font>"
                Exceptions.handle(e, ::log)
            }

            telemetry.opModeStatus = "<font color='green'><b>running</b></font>"
            Dbg.logd("BunyipsOpMode: starting activeLoop()...")
            while (opModeIsActive() && !operationsCompleted) {
                if (operationsPaused) {
                    // If the OpMode is paused, skip the loop and wait for the next hardware cycle
                    timer.update()
                    pushTelemetry()
                    // Slow down the OpMode as this is all we're doing
                    sleep(100)
                    continue
                }
                try {
                    // Run user-defined active loop
                    activeLoop()
                } catch (e: Exception) {
                    telemetry.overrideStatus = "<font color='red'><b>error</b></font>"
                    Exceptions.handle(e, ::log)
                }
                // Update telemetry and timers
                timer.update()
                pushTelemetry()
            }

            telemetry.opModeStatus = "<font color='gray'>finished</font>"
            try {
                onFinish()
            } catch (e: Exception) {
                telemetry.overrideStatus = "<font color='red'><b>error</b></font>"
                Exceptions.handle(e, ::log)
            }
            // overheadTelemetry will no longer update, will remain frozen on last value
            timer.update()
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
                // Save some CPU cycles
                sleep(500)
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
            // Ensure all threads have been told to stop
            gamepadExecutor?.shutdownNow()
            Threads.stopAll()
            try {
                onStop()
            } catch (e: Exception) {
                telemetry.overrideStatus = "<font color='red'><b>error</b></font>"
                Exceptions.handle(e, ::log)
            }
            _instance = null
            safeHaltHardware()
            // Telemetry may be not in a nice state, so we will call our stateful functions
            // such as thread stops and cleanup in onStop() first before updating the status
            telemetry.opModeStatus = "<font color='red'>terminating</font>"
            Dbg.logd("BunyipsOpMode: active cycle completed in ${timer.elapsedTime().inUnit(Seconds)} secs")
            pushTelemetry()
            Dbg.logd("BunyipsOpMode: exiting...")
        }
    }

    /**
     * Set a task that will run as an init-task. This will run
     * after your [onInit] has completed, allowing you to initialise hardware first.
     * This is an optional method, and runs alongside [onInitLoop].
     *
     * You should store any running variables inside the task itself, and keep the instance of the task
     * defined as a field in your OpMode. You can then use this value in your [onInitDone] to do
     * what you need to after the init-task has finished. This method should be paired with [onInitDone]
     * to do anything after the initTask has finished.
     *
     * If you do not define an initTask, then running it during the `dynamic_init` phase will be skipped.
     *
     * @see onInitDone
     */
    protected fun setInitTask(task: Task) {
        if (initTask != null) {
            Dbg.warn(javaClass, "Init-task has already been set to %, overriding it with %...", initTask, task)
        }
        initTask = task
    }

    // These telemetry methods exist for continuity as they used to be the primary way to use telemetry in BunyipsLib,
    // but have since moved to the DualTelemetry class. These methods are now simply aliases to the new methods.

    /**
     * Update and push queued [telemetry] to the Driver Station and FtcDashboard.
     */
    fun pushTelemetry() {
        telemetry.update()
    }

    /**
     * Add any additional telemetry to the FtcDashboard [telemetry] packet.
     */
    fun addDashboardTelemetry(key: String, value: Any?) {
        telemetry.addDashboard(key, value)
    }

    /**
     * Add any field overlay data to the FtcDashboard [telemetry] packet.
     */
    fun dashboardFieldOverlay(): Canvas {
        return telemetry.dashboardFieldOverlay()
    }

    /**
     * Add data to the [telemetry] object, with integrated formatting.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     * @return The telemetry [Item] added to the Driver Station, null if the send failed from overflow
     */
    fun addTelemetry(format: Any, vararg args: Any?): DualTelemetry.HtmlItem {
        return telemetry.add(format, *args)
    }

    /**
     * Add a data to the [telemetry] object, with integrated formatting.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     * @return The telemetry [Item] added to the Driver Station
     */
    fun addRetainedTelemetry(format: Any, vararg args: Any?): DualTelemetry.HtmlItem {
        return telemetry.addRetained(format, *args)
    }

    /**
     * Remove retained entries from the [telemetry] object.
     * @param items The items to remove from the telemetry object
     */
    fun removeRetainedTelemetry(vararg items: Item) {
        telemetry.removeRetained(*items)
    }

    /**
     * Remove retained entries from the [telemetry] object.
     * @param items The items to remove from the telemetry object
     */
    fun removeRetainedTelemetry(items: List<Item>) {
        removeRetainedTelemetry(*items.toTypedArray())
    }

    /**
     * Log a message to the [telemetry] log, with integrated formatting.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    fun log(format: Any, vararg args: Any?) {
        telemetry.log(format, *args)
    }

    /**
     * Log a message to the [telemetry] log, with integrated formatting.
     * @param obj Class where this log was called (name will be prepended to message)
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    fun log(obj: Class<*>, format: Any, vararg args: Any?) {
        telemetry.log(obj, format, *args)
    }

    /**
     * Log a message into the [telemetry] log
     * @param stck StackTraceElement with information about where this log was called (see Text.getCallingUserCodeFunction())
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    fun log(stck: StackTraceElement, format: Any, vararg args: Any?) {
        telemetry.log(stck, format, *args)
    }

    /**
     * Reset [telemetry] data, including retention and FtcDashboard
     */
    fun resetTelemetry() {
        telemetry.clearAll()
    }

    /**
     * Clear [telemetry] on the Driver Station, not including retention
     */
    fun clearTelemetry() {
        telemetry.clear()
    }

    /**
     * Reset member fields to their default values as the instance may be reused.
     */
    private fun resetFields() {
        operationsCompleted = false
        operationsPaused = false
        safeHaltHardwareOnStop = false
        gamepadExecutor = null
        RoadRunner.resetForOpMode()
    }

    /**
     * Call to manually finish the OpMode.
     * This is a dangerous method, as the OpMode will no longer be able to run any main thread code.
     * This method should be called when the OpMode is finished and no longer needs to run, and will
     * put the OpMode in a state where it will not run any more code (including timers & telemetry).
     * @param safeHaltHardwareOnStop If true (default), all motors/devices will be actively told to stop for the remainder of the OpMode.
     */
    @JvmOverloads
    fun finish(safeHaltHardwareOnStop: Boolean = true) {
        if (operationsCompleted) {
            return
        }
        this.safeHaltHardwareOnStop = safeHaltHardwareOnStop
        operationsCompleted = true
        Dbg.logd("BunyipsOpMode: activeLoop() terminated by finish().")
        telemetry.addRetained(
            "<b>Robot ${if (safeHaltHardwareOnStop) "<font color='green'>is stopped</font>" else "tasks finished"}. All operations completed.</b>",
            true
        )
        pushTelemetry()
    }

    /**
     * Call to command all motors and sensors on the robot to stop.
     * This method is continuously called when no OpMode is running, and allows you to do the same while still
     * in an OpMode (called internally after [finish]).
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
     * Call to temporarily halt all [activeLoop]-related updates from running.
     * Note this will pause the entire [activeLoop], but continue to update timers and telemetry. These events
     * must be handled manually if needed, which include any conditional calls to [resume].
     */
    fun halt() {
        if (operationsPaused) {
            return
        }
        operationsPaused = true
        telemetry.overrideStatus = "<font color='yellow'>halted</font>"
        Dbg.logd("BunyipsOpMode: activeLoop() halted.")
    }

    /**
     * Call to resume the [activeLoop] after a [halt] call.
     */
    fun resume() {
        if (!operationsPaused) {
            return
        }
        operationsPaused = false
        telemetry.overrideStatus = null
        Dbg.logd("BunyipsOpMode: activeLoop() resumed.")
    }

    /**
     * Dangerous method: call to shut down the OpMode as soon as possible.
     * This will run any [BunyipsOpMode] cleanup code, much as if the user pressed the `STOP` button.
     */
    fun exit() {
        Dbg.logd("BunyipsOpMode: exiting opmode...")
        finish()
        requestOpModeStop()
    }

    /**
     * Dangerous method: call to IMMEDIATELY terminate the OpMode.
     * **No further code will run**, and this should only be used in emergencies.
     */
    fun emergencyStop() {
        Dbg.logd("BunyipsOpMode: emergency stop requested.")
        terminateOpModeNow()
    }
}
