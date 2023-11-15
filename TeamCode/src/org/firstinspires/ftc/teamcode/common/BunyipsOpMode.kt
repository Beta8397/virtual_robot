package org.firstinspires.ftc.teamcode.common

//import org.firstinspires.ftc.teamcode.BuildConfig
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.Telemetry.Item
import org.firstinspires.ftc.teamcode.common.Text.formatString
import kotlin.math.roundToInt

/**
 * Base class for all OpModes that provides a number of useful methods and utilities for development.
 * Includes a lifecycle that is similar to an iterative lifecycle, but includes logging, error catching,
 * and abstraction to provide phased code execution.
 *
 * @author Lucas Bubner, 2023
 */
abstract class BunyipsOpMode : LinearOpMode() {
    var movingAverageTimer: MovingAverageTimer? = null
        private set

    private var operationsCompleted = false
    private var operationsPaused = false
    private var opModeStatus = "idle"

    private lateinit var overheadTelemetry: Item

    /**
     * FtcDashboard cannot be utilised during a match according to <RS09>, so this is only for development.
     * It is allowed during the Pits or Practice Field, or in any other non-match situation.
     * The dashboard can be disabled with the special Enable/Disable Dashboard OpMode.
     */
    // Dashboard capabilities is impossible in virtual robot
//    val dashboard: FtcDashboard = FtcDashboard.getInstance()
//    private var queuedPacket = TelemetryPacket()

    /**
     * One-time setup for operations that need to be done for every OpMode
     * This method is not exception protected!
     */
    private fun setup() {
        // At the moment, FtcDashboard hasn't implemented some methods, so we have them commented out for now
        // This does not apply to the normal Driver Station telemetry
        telemetry.log().displayOrder = Telemetry.Log.DisplayOrder.OLDEST_FIRST
        // dashboard?.telemetry?.log()?.displayOrder = Telemetry.Log.DisplayOrder.OLDEST_FIRST
        telemetry.captionValueSeparator = ""
        // dashboard?.telemetry?.captionValueSeparator = ""
        // Uncap the telemetry log limit to ensure we capture everything
        telemetry.log().capacity = 999999
        // dashboard?.telemetry?.log()?.capacity = 999999
        movingAverageTimer = MovingAverageTimer(100)

        overheadTelemetry =
            telemetry.addData("", "BOM: idle | T+0s | 0.000ms | u1: (?) | u2: (?)\n").setRetained(true)
        pushTelemetry()
    }

    /**
     * Runs upon the pressing of the INIT button on the Driver Station.
     * This is where your hardware should be initialised.
     */
    protected abstract fun onInit()

    /**
     * Run code in a loop AFTER onInit has completed, until
     * start is pressed on the Driver Station or true is returned to this method.
     * If not implemented, the OpMode will continue on as normal and wait for start.
     */
    protected open fun onInitLoop(): Boolean {
        return true
    }

    /**
     * Allow code to execute once after all initialisation has finished.
     * Note: this method is always called even if initialisation is cut short by the driver station.
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
     * Code to run when the START button is pressed on the Driver Station.
     * This method will be called on each hardware cycle.
     */
    protected abstract fun activeLoop()

    /**
     * Perform one time clean-up operations after the OpMode finishes.
     * This method is not exception protected!
     */
    protected open fun onStop() {
    }

    /**
     * Main method overridden from LinearOpMode that handles the OpMode lifecycle.
     * @throws InterruptedException
     */
    @Throws(InterruptedException::class)
    final override fun runOpMode() {
        try {
//            Dbg.log("=============== BunyipsOpMode ${BuildConfig.GIT_COMMIT} ${BuildConfig.GIT_BRANCH} ${BuildConfig.BUILD_TIME} sdk${BuildConfig.VERSION_NAME} ===============")
            // Not pushing to FtcDashboard as it is in Logcat
            telemetry.log()
                .add("bunyipsopmode abc123-dirty virtual_robot xx-xx-xx@xx:xx:xx sdk8.0.0")
            updateOpModeStatus("setup")
            Dbg.log("BunyipsOpMode: setting up...")
            // Run BunyipsOpMode setup
            setup()

            updateOpModeStatus("static_init")
            Dbg.log("BunyipsOpMode: firing onInit()...")
            // Store telemetry objects raised by onInit() by turning off auto-clear
            setTelemetryAutoClear(false)
            pushTelemetry()
//            if (!gamepad1.atRest() || !gamepad2.atRest()) {
//                log("warning: a gamepad was not zeroed during init. please ensure controllers zero out correctly.")
//            }
            // Run user-defined setup
            try {
                onInit()
            } catch (e: Throwable) {
                // All InterruptedExceptions are handled by the FTC SDK and are raised in ErrorUtil
                ErrorUtil.handleCatchAllException(e, ::log)
            }

            updateOpModeStatus("dynamic_init")
            pushTelemetry()
            Dbg.log("BunyipsOpMode: starting onInitLoop()...")
            // Run user-defined dynamic initialisation
            while (!isStopRequested && !isStarted) {
                try {
                    // Run until onInitLoop returns true or the OpMode is continued
                    if (onInitLoop()) break
                } catch (e: Throwable) {
                    ErrorUtil.handleCatchAllException(e, ::log)
                }
                movingAverageTimer?.update()
                pushTelemetry()
            }

            updateOpModeStatus("finish_init")
            Dbg.log("BunyipsOpMode: firing onInitDone()...")
            // Run user-defined final initialisation
            try {
                onInitDone()
            } catch (e: Throwable) {
                ErrorUtil.handleCatchAllException(e, ::log)
            }

            // Ready to go.
            updateOpModeStatus("ready")
            telemetry.addData("BUNYIPSOPMODE : ", "INIT COMPLETE -- PLAY WHEN READY.")
//            queuedPacket.put("BUNYIPSOPMODE", "INIT COMPLETE -- PLAY WHEN READY.")
            Dbg.log("BunyipsOpMode: ready.")
            // Set telemetry to an inert state while we wait for start
            pushTelemetry()
            overheadTelemetry.setValue("BOM: ready | T+0s | 0.000ms | u1: (?) | u2: (?)\n")

            waitForStart()

            // Play button has been pressed
            updateOpModeStatus("starting")
            setTelemetryAutoClear(true)
            clearTelemetry()
            movingAverageTimer?.reset()
            Dbg.log("BunyipsOpMode: firing onStart()...")
            try {
                // Run user-defined start operations
                onStart()
            } catch (e: Throwable) {
                ErrorUtil.handleCatchAllException(e, ::log)
            }

            updateOpModeStatus("running")
            Dbg.log("BunyipsOpMode: starting activeLoop()...")
            while (opModeIsActive() && !operationsCompleted) {
                if (operationsPaused) {
                    // If the OpMode is paused, skip the loop and wait for the next hardware cycle
                    // Timers and telemetry will remain frozen and have to be controlled externally
                    idle()
                    continue
                }
                try {
                    // Run user-defined active loop
                    activeLoop()
                } catch (e: Throwable) {
                    ErrorUtil.handleCatchAllException(e, ::log)
                }
                // Update telemetry and timers
                movingAverageTimer?.update()
                pushTelemetry()
            }

            updateOpModeStatus("finished")
            // overheadTelemetry will no longer update, will remain frozen on last value
            pushTelemetry()
            Dbg.log("BunyipsOpMode: finished.")
            // Wait for user to hit stop or for the OpMode to be terminated
            while (opModeIsActive()) {
                idle()
            }
        } catch (t: Throwable) {
            Dbg.error("BunyipsOpMode: unhandled throwable! <${t.message}>")
            Dbg.sendStacktrace(t)
            // As this error occurred somewhere not inside user code, we should not swallow it
            throw t
        } finally {
            Dbg.log("BunyipsOpMode: cleaning up...")
            updateOpModeStatus("terminating")
            onStop()
            Dbg.log("BunyipsOpMode: exiting...")
        }
    }

    private fun updateOpModeStatus(newStatus: String) {
        log("status changed: $opModeStatus >> $newStatus")
        opModeStatus = newStatus
    }

    /**
     * Update telemetry for both the Driver Station and the FtcDashboard
     * This will only push dashboard changes made through addTelemetry() and log() calls.
     */
    fun pushTelemetry() {
        telemetry.update()
//        dashboard.sendTelemetryPacket(queuedPacket)
        // Reset the queued packet
//        queuedPacket = TelemetryPacket()

        // Requeue new overhead status message
        // This is only showed on the Driver Station for brevity
        overheadTelemetry.setValue(
            "BOM: $opModeStatus | T+${
                movingAverageTimer?.elapsedTime()?.div(1000)?.roundToInt() ?: 0
            }s | ${movingAverageTimer?.movingAverageString() ?: "0.000ms"} | u1: ${
                Controller.movementString(gamepad1)
            } | u2: ${
                Controller.movementString(gamepad2)
            }\n"
        )
    }

    private fun makeTelemetryObject(value: String): Item {
        // Create a clone object to insert into the dashboard
//        queuedPacket.put("T+${movingAverageTimer?.elapsedTime()?.div(1000)?.roundToInt() ?: 0}s", value)
        return telemetry.addData("", value)
    }

    /**
     * Add data to the telemetry object
     * @param value A string to add to telemetry
     * @return The telemetry item added to the Driver Station
     */
    fun addTelemetry(value: String): Item {
        return makeTelemetryObject(value)
    }

    /**
     * Add data to the telemetry object using a custom format string
     * @param fstring A format string to add to telemetry
     * @param objs The objects to format into the string
     * @return The telemetry item added to the Driver Station
     */
    fun addTelemetry(fstring: String, vararg objs: Any): Item {
        if (objs.isEmpty()) {
            return addTelemetry(fstring)
        }
        return addTelemetry(formatString(fstring, objs.asList()))
    }

    /**
     * Add retained non-auto-clearing data to the telemetry object
     * @param value A string to add to telemetry
     * @return The telemetry item added to the Driver Station
     */
    fun addRetainedTelemetry(value: String): Item {
        return makeTelemetryObject(value).setRetained(true)
    }

    /**
     * Add a data to the telemetry object using a custom format string
     * @param fstring A format string to add to telemetry
     * @param objs The objects to format into the string
     * @return The telemetry item added to the Driver Station
     */
    fun addRetainedTelemetry(fstring: String, vararg objs: Any): Item {
        if (objs.isEmpty()) {
            return addRetainedTelemetry(fstring)
        }
        return addRetainedTelemetry(formatString(fstring, objs.asList()))
    }

    /**
     * Log a message to the telemetry log
     * @param message The message to log
     */
    fun log(message: String) {
        telemetry.log().add(message)
//        dashboard.telemetry.log().add(message)
    }

    /**
     * Log a message to the telemetry log using a format string
     * @param fstring A format string to add to telemetry
     * @param objs The objects to format into the string
     */
    fun log(fstring: String, vararg objs: Any) {
        if (objs.isEmpty()) {
            return log(fstring)
        }
        return log(formatString(fstring, objs.asList()))
    }

    /**
     * Remove an entry from the telemetry object. Useful for removing retained data without clearing.
     * This method should be combined with the return value of addTelemetry.
     * Note this method is not needed if you are using unretained data, as it will be cleared automatically
     * if auto-clear is enabled, which it is by default.
     * @param items The telemetry items to remove
     */
    fun removeTelemetryItems(vararg items: Item) {
        for (item in items) {
            val res = telemetry.removeItem(item)
            if (!res) {
                log("failed to remove telemetry item: $item")
            }
        }
    }

    fun removeTelemetryItems(items: List<Item>) {
        removeTelemetryItems(*items.toTypedArray())
    }

    /**
     * Reset telemetry data, including retention
     */
    fun resetTelemetry() {
        telemetry.clearAll()
        // Must reassign the overhead telemetry item
        overheadTelemetry =
            telemetry.addData("", "BOM: unknown | T+?s | ?ms | u1: (?) | u2: (?)").setRetained(true)
//        dashboard.clearTelemetry()
    }

    /**
     * Clear telemetry on screen, not including retention
     */
    fun clearTelemetry() {
        telemetry.clear()
//        dashboard.clearTelemetry()
    }

    /**
     * Set telemetry auto clear status
     */
    fun setTelemetryAutoClear(autoClear: Boolean) {
        // Auto clear does not apply to the dashboard
        telemetry.isAutoClear = autoClear
    }

    /**
     * Get auto-clear status of telemetry
     */
    fun getTelemetryAutoClear(): Boolean {
        return telemetry.isAutoClear
    }

    fun getSDKTelemetry(): Telemetry {
        return super.telemetry
    }

//    fun getDashboardTelemetry(): Telemetry {
////        return dashboard.telemetry
//    }

    /**
     * Call to prevent hardware loop from calling activeLoop(), indicating an OpMode that is finished.
     * This is a dangerous method, as the OpMode will no longer be able to run any main thread code.
     */
    fun finish() {
        if (operationsCompleted) {
            return
        }
        operationsCompleted = true
        Dbg.log("BunyipsOpMode: activeLoop() terminated by finish().")
        telemetry.addData("BUNYIPSOPMODE : ", "activeLoop terminated. All operations completed.")
//        dashboard.telemetry?.addData("BUNYIPSOPMODE : ", "activeLoop terminated. All operations completed.")
        pushTelemetry()
    }

    /**
     * Call to temporarily halt all activeLoop-related updates from running.
     * Note this will pause all MovingAverageTimer and telemetry updates. These events
     * must be handled manually if needed, which include any conditional calls to resume().
     */
    fun halt() {
        if (operationsPaused) {
            return
        }
        operationsPaused = true
        updateOpModeStatus("halted")
        Dbg.log("BunyipsOpMode: activeLoop() halted.")
        telemetry.addData("BUNYIPSOPMODE : ", "activeLoop halted. Operations paused.")
//        dashboard.telemetry?.addData("BUNYIPSOPMODE : ", "activeLoop halted. Operations paused.")
        pushTelemetry()
    }

    /**
     * Call to resume the activeLoop after a halt() call.
     */
    fun resume() {
        if (!operationsPaused) {
            return
        }
        operationsPaused = false
        updateOpModeStatus("running")
        Dbg.log("BunyipsOpMode: activeLoop() resumed.")
        telemetry.addData("BUNYIPSOPMODE : ", "activeLoop resumed. Operations resumed.")
//        dashboard.telemetry?.addData("BUNYIPSOPMODE : ", "activeLoop resumed. Operations resumed.")
        pushTelemetry()
    }

    /**
     * Dangerous method: call to shut down the OpMode as soon as possible.
     * This will run any BunyipsOpMode cleanup code.
     */
    fun exit() {
        Dbg.log("BunyipsOpMode: exiting opmode...")
        finish()
        requestOpModeStop()
    }

    /**
     * Dangerous method: call to IMMEDIATELY terminate the OpMode.
     * This will not run any cleanup code, and should only be used in emergencies.
     */
    fun emergencyStop() {
        Dbg.log("BunyipsOpMode: emergency stop requested.")
//        terminateOpModeNow()
    }
}