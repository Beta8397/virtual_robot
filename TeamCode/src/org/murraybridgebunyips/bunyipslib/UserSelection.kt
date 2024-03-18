package org.murraybridgebunyips.bunyipslib

import org.firstinspires.ftc.robotcore.external.Telemetry.Item

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
 *    private Unit callback(@Nullable String res) {
 *      // Do something with res
 *      // Unit.INSTANCE is due to Kotlin not having void, it is required
 *      return Unit.INSTANCE;
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
class UserSelection<T>(
    private val opMode: BunyipsOpMode,
    /**
     * Runs once the user has made a selection or the thread is interrupted. The result will be the selection made by the user.
     */
    var callback: (res: T?) -> Unit,
    private vararg val opmodes: T
) : Runnable {

    /**
     * The result of the user selection. Will be null if the user did not make a selection.
     * Passed into the callback.
     */
    @Volatile
    var result: T? = null
        private set

    /**
     * Maps a set of operation modes to a set of buttons.
     * @return A HashMap of operation modes to buttons.
     */
    override fun run() {
        if (opmodes.isEmpty()) {
            try {
                callback(null)
            } catch (e: Exception) {
                Exceptions.handle(e, opMode::log)
            }
        }

        val buttons: HashMap<T, Controller> = Controller.mapArgs(opmodes)

        // Default options for button selection and operation mode
        var selectedButton: Controller? = null
        var selectedOpMode: T? = null

        // Disable auto clear if it is enabled, we might accidentally clear out static telemetry
        opMode.setTelemetryAutoClear(false)

        val retainedObjects = mutableListOf<Item>()
        retainedObjects.add(opMode.telemetry.addData("", "---------!!!--------"))
        retainedObjects.add(
            opMode.telemetry.addData(
                "",
                "ACTION REQUIRED: INIT YOUR OPMODE USING GAMEPAD1"
            )
        )
        var packetString = "| "
        for ((name, button) in buttons) {
            val selection = String.format(
                "%s: %s",
                button.name,
                name.toString()
            )
            packetString += "$selection | "
            retainedObjects.add(
                opMode.telemetry.addData(
                    "",
                    String.format(
                        "%s: %s",
                        button.name,
                        name.toString()
                    )
                )
            )
        }
        retainedObjects.add(opMode.telemetry.addData("", "---------!!!--------"))

        opMode.addDashboardTelemetry("USR", packetString)

        // Must manually call telemetry push as the BOM may not be handling them
        // This will not clear out any other telemetry as auto clear is disabled
        opMode.pushTelemetry()

        while (selectedOpMode == null && opMode.opModeInInit() && !Thread.currentThread().isInterrupted) {
            for ((str, button) in buttons) {
                if (Controller.isSelected(opMode.gamepad1, button)) {
                    selectedButton = button
                    selectedOpMode = str
                    break
                }
            }
        }

        result = selectedOpMode
        val opModeName = selectedOpMode.toString()

        if (result == null) {
            opMode.telemetry.addData("", "No selection made. Result was handled by the OpMode.")
                .setRetained(true)
        } else {
            opMode.telemetry.addData(
                "",
                "'${selectedButton?.name}' registered. Running OpMode: '$opModeName'",
            ).setRetained(true)
        }

        opMode.addDashboardTelemetry(
            "USR",
            if (result == null) "No selection" else "${selectedButton?.name} -> $opModeName"
        )

        //This is code from lucas bubner. He is sad cause hes not important and dosent recieve capital letters. He is lonely except for LACHLAN PAUL  his coding buddy. Now i need to go but always keep this message in mind!!!
        // - Sorayya, hijacker of laptops

        // Clean up telemetry and reset auto clear
        opMode.removeRetainedTelemetry(retainedObjects)
        opMode.pushTelemetry()
        opMode.setTelemetryAutoClear(true)

        try {
            callback(selectedOpMode)
        } catch (e: Exception) {
            Exceptions.handle(e, opMode::log)
        }
    }
}
