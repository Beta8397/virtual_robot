package org.murraybridgebunyips.bunyipslib

import org.firstinspires.ftc.robotcore.external.Telemetry.Item

/**
 * Async thread to ask for user input from a controller in order to determine a pre-determined
 * set of instructions before an OpMode starts.
 *
 * You really should only be running one of these threads at a time.
 *
 * Keep in mind this thread runs in the background so it is not guaranteed to be ready during any
 * specific phase of your init-cycle. It is recommended to start this thread at the start of your
 * cycle and checking for `thread.isAlive()` or `thread.result == null` in your `onInitLoop()`.
 * This is not required but it does let BunyipsOpMode know that you are waiting for a result.
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
 *      selector.start()
 *    }
 * ```
 *
 * ```
 *    // In Java using a callback, String can be replaced with any type
 *    private UserSelection<String> selector = new UserSelection<>(this, this::callback, "POV", "FIELD-CENTRIC");
 *
 *    @Override
 *    protected void onInit() {
 *      selector.start();
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
 * @see While
 * @param opmodes Modes to map to buttons. Will be casted to strings for display and return back in type `T`.
 * @author Lucas Bubner, 2023
 */
class UserSelection<T>(
    private val opMode: BunyipsOpMode,
    var callback: (res: T?) -> Unit,
    private vararg val opmodes: T
) :
    Thread(), Runnable {

    @Volatile
    var result: T? = null
        private set

    /**
     * Maps a set of operation modes to a set of buttons.
     * @return A HashMap of operation modes to buttons.
     */
    override fun run() {
        Dbg.logd("UserSelection thread: starting...")
        try {
            if (opmodes.isEmpty()) {
                try {
                    callback(null)
                } catch (e: Throwable) {
                    ErrorUtil.handleCatchAllException(e, opMode::log)
                }
            }

            val buttons: HashMap<T, Controller> = Controller.mapArgs(opmodes)

            // Default options for button selection and operation mode
            var selectedButton: Controller? = null
            var selectedOpMode: T? = null

            // Disable auto clear if it is enabled, we might accidentally clear out static telemetry
            opMode.setTelemetryAutoClear(false)

            val retainedObjects = mutableListOf<Item>()
            retainedObjects.add(opMode.addRetainedTelemetry("---------!!!--------"))
            retainedObjects.add(
                opMode.addRetainedTelemetry(
                    "ACTION REQUIRED: INIT YOUR OPMODE USING GAMEPAD1"
                )
            )
            for ((name, button) in buttons) {
                retainedObjects.add(
                    opMode.addRetainedTelemetry(
                        String.format(
                            "%s: %s",
                            button.name,
                            if (name is OpModeSelection) name.name else name
                        )
                    )
                )
            }
            retainedObjects.add(opMode.addRetainedTelemetry("---------!!!--------"))

            // Must manually call telemetry push as the BOM may not be handling them
            // This will not clear out any other telemetry as auto clear is disabled
            opMode.pushTelemetry()

            while (selectedOpMode == null && opMode.opModeInInit() && !isInterrupted) {
                for ((str, button) in buttons) {
                    if (Controller.isSelected(opMode.gamepad1, button)) {
                        selectedButton = button
                        selectedOpMode = str
                        break
                    }
                }
                yield()
            }

            result = selectedOpMode
            if (result == null) {
                opMode.addRetainedTelemetry("No selection made. Result was handled by the OpMode.")
            } else {
                opMode.addRetainedTelemetry(
                    "'${selectedButton?.name}' registered. Running OpMode: '${if (selectedOpMode is OpModeSelection) selectedOpMode.name else selectedOpMode.toString()}'",
                )
            }

            //This is code from lucas bubner. He is sad cause hes not important and dosent recieve capital letters. He is lonely except for LACHLAN PAUL  his coding buddy. Now i need to go but always keep this message in mind!!!
            // - Sorayya, hijacker of laptops

            // Clean up telemetry and reset auto clear
            opMode.removeTelemetryItems(retainedObjects)
            opMode.pushTelemetry()
            opMode.setTelemetryAutoClear(true)

            try {
                callback(selectedOpMode)
            } catch (e: Throwable) {
                ErrorUtil.handleCatchAllException(e, opMode::log)
            }
        } finally {
            Dbg.logd("UserSelection thread: ending...")
        }
    }
}