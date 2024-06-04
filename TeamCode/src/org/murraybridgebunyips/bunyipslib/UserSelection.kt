package org.murraybridgebunyips.bunyipslib

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry.Item
import org.murraybridgebunyips.bunyipslib.Text.round
import org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds
import java.util.function.Consumer

/**
 * Async thread to ask for user input from a controller in order to determine a pre-determined
 * set of instructions before an OpMode starts.
 *
 * You really should only be running one of these threads at a time, preferably using the Threads
 * class to start and manage it to allow for logging and OpMode management.
 *
 * Keep in mind this thread runs in the background so it is not guaranteed to be ready during any
 * specific phase of your init-cycle. It is recommended to check if this thread is running using
 * `Threads.isRunning(selector)` in your `onInitLoop()` to ensure BOM knows it has to wait for the
 * user to make a selection. If you do not do this, the OpMode will assume it is ready to run regardless.
 *
 * The result of this thread will be stored in the `result` property, which you can access yourself
 * or you can attach a callback to the `callback` property to be run once the thread is complete.
 * This callback will still be run if the OpMode moves to a running state without a selection. In
 * the event a user does not make a selection, the callback result and `result` property will be
 * null.
 *
 * ```
 *    // In Kotlin using a lambda function, String can be replaced with any type
 *    private val selector: UserSelection<String> = UserSelection(this, { if (it == "POV") initPOVDrive() else initFCDrive() }, "POV", "FIELD-CENTRIC")
 *
 *    override fun onInit() {
 *      Threads.run(selector)
 *    }
 * ```
 *
 * ```
 *    // In Java using a callback, String can be replaced with any type
 *    private UserSelection<String> selector = new UserSelection<>(this, this::callback, "POV", "FIELD-CENTRIC");
 *
 *    @Override
 *    protected void onInit() {
 *      Threads.run(selector);
 *    }
 *
 *    private void callback(@Nullable String res) {
 *      // Do something with res
 *    }
 * ```
 *
 * res will be null if the user did not make a selection.
 *
 * Updated to use dynamic button mapping and generics 04/08/23.
 * Updated to be async and removed time restriction 07/09/23.
 *
 * @param opmodes Modes to map to buttons. Will be casted to strings for display and return back in type `T`.
 * @author Lucas Bubner, 2023
 */
class UserSelection<T : Any>(
    private val opMode: BunyipsOpMode,
    /**
     * Runs once the user has made a selection or the thread is interrupted. The result will be the selection made by the user.
     */
    private val callback: Consumer<T?>,
    private vararg val opmodes: T
) : Runnable {
    private val timer = ElapsedTime()

    /**
     * The result of the user selection. Will be null if the user did not make a selection.
     * Passed into the callback.
     */
    @Volatile
    var result: T? = null
        private set

    /**
     * The button that was selected by the user.
     */
    var selectedButton: Controls = Controls.NONE
        private set

    /**
     * Maps a set of operation modes to a set of buttons.
     * @return A HashMap of operation modes to buttons.
     */
    override fun run() {
        if (opmodes.isEmpty()) {
            try {
                callback.accept(null)
            } catch (e: Exception) {
                Exceptions.handle(e, opMode.telemetry::log)
            }
        }

        val buttons: HashMap<T, Controls> = Controls.mapArgs(opmodes)

        // Default options for button selection and operation mode
        var selectedOpMode: T? = null

        // Disable auto clear if it is enabled, we might accidentally clear out static telemetry
        opMode.telemetry.isAutoClear = false

        val retainedObjects = mutableListOf<Item>()
        retainedObjects.add(opMode.telemetry.addDS("<b>---------<font color='red'>!!!</font>--------</b>"))
        retainedObjects.add(
            opMode.telemetry.addDS(
                "<font color='yellow'><b>ACTION REQUIRED</b></font>: INIT OPMODE WITH GAMEPAD 1"
            )
        )
        var packetString = "<font color='gray'>|</font> "
        for ((name, button) in buttons) {
            val selection = String.format(
                "%s: %s",
                button.name,
                name.toString()
            )
            packetString += "$selection <font color='gray'>|</font> "
            retainedObjects.add(
                opMode.telemetry.addDS(
                    "| %: <b>%</b>",
                    button.name,
                    StartingPositions.getHTMLIfAvailable(name)
                )
            )
        }
        retainedObjects.add(opMode.telemetry.addDS("<b>---------<font color='red'>!!!</font>--------</b>"))

        opMode.telemetry.addDashboard("<small>USR</small>", packetString)

        // Must manually call telemetry push as the BOM may not be handling them
        // This will not clear out any other telemetry as auto clear is disabled
        opMode.telemetry.update()

        var flash = false
        while (selectedOpMode == null && opMode.opModeInInit() && !Thread.currentThread().isInterrupted) {
            for ((str, button) in buttons) {
                if (Controls.isSelected(opMode.gamepad1, button)) {
                    selectedButton = button
                    selectedOpMode = str
                    break
                }
            }
            if (timer.seconds() > 0.5) {
                flash = !flash
                timer.reset()
            }
            if (flash) {
                retainedObjects[0].setValue("<b>---------<font color='red'>!!!</font>--------</b>")
                retainedObjects[retainedObjects.size - 1].setValue("<b>---------<font color='red'>!!!</font>--------</b>")
            } else {
                retainedObjects[0].setValue("<b><font color='red'>---------</font><font color='white'>!!!</font><font color='red'>--------</font></b>")
                retainedObjects[retainedObjects.size - 1].setValue("<b><font color='red'>---------</font><font color='white'>!!!</font><font color='red'>--------</font></b>")
            }
        }

        result = selectedOpMode
        val opModeName = selectedOpMode.toString()

        if (result == null) {
            opMode.telemetry.log("<font color='yellow'>No user OpMode selection was made.</font>")
        } else {
            opMode.telemetry.log(
                "Running OpMode: <font color='#caabff'>${selectedButton.name} -> <b>$opModeName</b></font>",
            )
        }

        opMode.telemetry.addDashboard(
            "<small>USR</small>",
            if (result == null) "No selection" else "${selectedButton.name} -> $opModeName@T+${
                round(
                    opMode.timer.elapsedTime().inUnit(Seconds), 1
                )
            }s"
        )

        //This is code from lucas bubner. He is sad cause hes not important and dosent recieve capital letters. He is lonely except for LACHLAN PAUL  his coding buddy. Now i need to go but always keep this message in mind!!!
        // - Sorayya, hijacker of laptops

        // Clean up telemetry and reset auto clear
        opMode.telemetry.remove(retainedObjects)
        opMode.telemetry.isAutoClear = true

        try {
            callback.accept(selectedOpMode)
        } catch (e: Exception) {
            Exceptions.handle(e, opMode.telemetry::log)
        }
    }
}
