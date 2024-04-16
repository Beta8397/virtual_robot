package org.murraybridgebunyips.bunyipslib

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.Func
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.Telemetry.DisplayFormat
import org.firstinspires.ftc.robotcore.external.Telemetry.Item
import org.murraybridgebunyips.bunyipslib.Text.formatString
import org.murraybridgebunyips.bunyipslib.external.units.Units.Milliseconds
import org.murraybridgebunyips.bunyipslib.external.units.Units.Second
import org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds
import kotlin.math.roundToInt

/**
 * Telemetry implementation for BunyipsLib, integrating FtcDashboard and Driver Station calls in one object, while
 * providing additional features useful for debugging and telemetry management.
 * This is used internally by [BunyipsOpMode] to be accessible by the overridden `telemetry` field.
 *
 * @author Lucas Bubner, 2024
 */
class DualTelemetry @JvmOverloads constructor(
    private val opMode: OpMode,
    private val movingAverageTimer: MovingAverageTimer? = null,
    /**
     * A tag to prepend to the overhead telemetry status message.
     */
    private val overheadTag: String? = null,
    /**
     * A string to display as the first log message in the telemetry log.
     */
    infoString: String? = null
) : Telemetry {
    private lateinit var overheadTelemetry: Item
    private var telemetryQueue = 0
    private val telemetryItems = mutableSetOf<Pair<ItemType, String>>()
    private var packet: TelemetryPacket? = TelemetryPacket()

    /**
     * A string to display the current 'status' of the OpMode, used for overhead telemetry.
     */
    var opModeStatus = "idle"

    private enum class ItemType {
        TELEMETRY,
        RETAINED_TELEMETRY,
        LOG
    }

    init {
        opMode.telemetry.log().displayOrder = Telemetry.Log.DisplayOrder.OLDEST_FIRST
        opMode.telemetry.captionValueSeparator = ""
        // Uncap the telemetry log limit to ensure we capture everything
        opMode.telemetry.log().capacity = 999999
        // Separate log from telemetry on the DS with an empty line
        opMode.telemetry.log().add("")
        if (infoString != null) {
            telemetryItems.add(
                Pair(
                    ItemType.LOG,
                    infoString
                )
            )
            opMode.telemetry.log().add(infoString)
        }
    }

    /**
     * Add data to the telemetry object for the Driver Station and FtcDashboard, with integrated formatting.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     * @return The telemetry item added to the Driver Station, null if the send failed from overflow
     */
    fun add(format: Any, vararg args: Any?): Item? {
        flushTelemetryQueue()
        if (telemetryQueue >= 255 && !opMode.telemetry.isAutoClear) {
            // Auto flush will fail as clearing is not permitted
            // We will send telemetry to the debugger instead as a fallback
            Dbg.log("Telemetry overflow: $format")
            return null
        }
        return createTelemetryItem(formatString(format.toString(), *args), false)
    }

    /**
     * Add a data to the telemetry object for the Driver Station and FtcDashboard, with integrated formatting.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     * @return The telemetry item added to the Driver Station
     */
    fun addRetained(format: Any, vararg args: Any?): Item {
        flushTelemetryQueue()
        // Retained objects will never be cleared, so we can just add them immediately, as usually
        // retained messages should be important and not be discarded
        return createTelemetryItem(formatString(format.toString(), *args), true)
    }

    /**
     * Add any additional telemetry to the Driver Station telemetry object.
     */
    fun addDS(format: Any, vararg args: Any?): Item {
        return opMode.telemetry.addData("", formatString(format.toString(), *args))
    }

    /**
     * Add any additional telemetry to the FtcDashboard telemetry packet.
     */
    fun addDashboard(key: String, value: Any?) {
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
     * Remove retained entries from the telemetry object.
     * @param items The items to remove from the telemetry object
     */
    fun removeRetained(vararg items: Item): Boolean {
        var ok = true
        for (item in items) {
            // We will be able to remove the item from the DS, but objects on FtcDashboard cannot
            // be removed as we no longer know the index of the object or the contents of the item.
            // This means all RT objects on the dashboard are permanent, and will respect their
            // last updated value. This is not a problem as retained objects are usually important
            // and can serve as a debugging tool.
            val res = opMode.telemetry.removeItem(item)
            if (!res) {
                Dbg.logd("Could not find telemetry item to remove: $item")
                ok = false
            }
        }
        return ok
    }

    /**
     * Remove retained entries from the telemetry object.
     * @param items The items to remove from the telemetry object
     */
    fun removeRetained(items: List<Item>) {
        removeRetained(*items.toTypedArray())
    }

    /**
     * Log a message to the telemetry log, with integrated formatting.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    fun log(format: Any, vararg args: Any?) {
        val fstring = formatString(format.toString(), *args)
        opMode.telemetry.log().add(fstring)
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
        opMode.telemetry.log().add(msg)
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
        opMode.telemetry.log().add(msg)
        telemetryItems.add(Pair(ItemType.LOG, msg))
    }

    /**
     * Update and push queued telemetry to the Driver Station and FtcDashboard.
     */
    override fun update(): Boolean {
        // Update main DS telemetry
        val retVal = opMode.telemetry.update()

        // Requeue new overhead status message
        val loopTime =
            if (movingAverageTimer != null) Text.round(movingAverageTimer.movingAverage(Milliseconds), 2) else 0.0
        val loopsSec = if (movingAverageTimer != null)
            if (!movingAverageTimer.loopsPer(Second).isNaN())
                Text.round(movingAverageTimer.loopsPer(Second), 1)
            else 0.0
        else 0.0
        val elapsedTime = movingAverageTimer?.elapsedTime(Seconds)?.roundToInt() ?: "?"

        val overheadStatus =
            "$opModeStatus | T+${elapsedTime}s | ${
                if (loopTime <= 0.0) {
                    if (loopsSec > 0) "$loopsSec l/s" else "?ms"
                } else {
                    "${loopTime}ms"
                }
            } | ${Controls.movementString(opMode.gamepad1)} ${Controls.movementString(opMode.gamepad2)}\n"

        overheadTelemetry.setValue("${if (overheadTag != null) "$overheadTag: " else ""}$overheadStatus")

        // FtcDashboard
        if (packet == null) {
            packet = TelemetryPacket()
        }

        packet?.let {
            it.put(overheadTag ?: "status", overheadStatus + "\n")

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

        if (opMode.telemetry.isAutoClear) {
            telemetryQueue = 0
            clearTelemetryObjects()
        }

        return retVal
    }

    /**
     * Clear telemetry on the Driver Station, not including retention
     */
    override fun clear() {
        telemetryQueue = 0
        clearTelemetryObjects()
        opMode.telemetry.clear()
    }

    /**
     * Reset telemetry data, including retention and FtcDashboard
     */
    override fun clearAll() {
        opMode.telemetry.clearAll()
        telemetryItems.clear()
        FtcDashboard.getInstance().clearTelemetry()
        telemetryQueue = 0
        overheadTelemetry = opMode.telemetry.addData(
            "",
            "${if (overheadTag != null) "$overheadTag: " else ""}unknown | T+?s | ?ms | (?) (?))\n"
        )
            .setRetained(true)
    }

    /**
     * Add an action to perform before Driver Station telemetry is updated.
     */
    override fun addAction(action: Runnable): Any {
        return opMode.telemetry.addAction(action)
    }

    /**
     * Remove an action from the telemetry object.
     */
    override fun removeAction(token: Any): Boolean {
        return opMode.telemetry.removeAction(token)
    }

    /**
     * Speak a message to the Driver Station.
     */
    override fun speak(text: String) {
        opMode.telemetry.speak(text)
    }

    /**
     * Speak a message to the Driver Station with language and country code.
     */
    override fun speak(text: String, languageCode: String, countryCode: String) {
        opMode.telemetry.speak(text, languageCode, countryCode)
    }

    /**
     * Whether the telemetry object is set to auto-clear after each update for the Driver Station.
     * FtcDashboard will always be false.
     */
    override fun isAutoClear(): Boolean {
        return opMode.telemetry.isAutoClear
    }

    /**
     * Set the telemetry object to auto-clear after each update for the Driver Station.
     */
    override fun setAutoClear(autoClear: Boolean) {
        opMode.telemetry.isAutoClear = autoClear
    }

    /**
     * Get the current transmission interval for the Driver Station.
     * FtcDashboard interval can be checked with `FtcDashboard.getInstance().getTelemetryTransmissionInterval()`.
     */
    override fun getMsTransmissionInterval(): Int {
        return opMode.telemetry.msTransmissionInterval
    }

    /**
     * Set the transmission interval in milliseconds for the Driver Station and FtcDashboard.
     */
    override fun setMsTransmissionInterval(msTransmissionInterval: Int) {
        opMode.telemetry.msTransmissionInterval = msTransmissionInterval
        FtcDashboard.getInstance().telemetryTransmissionInterval = msTransmissionInterval
    }

    /**
     * Get the current caption value separator for the Driver Station. This should always be an empty string as
     * BunyipsTelemtry does not use captions.
     */
    override fun getCaptionValueSeparator(): String {
        return opMode.telemetry.captionValueSeparator
    }

    /**
     * Get the current display format for the Driver Station.
     */
    override fun setDisplayFormat(displayFormat: DisplayFormat) {
        opMode.telemetry.setDisplayFormat(displayFormat)
    }

    /**
     * Setup and reset the telemetry object for the start of an OpMode.
     */
    fun setup() {
        clearAll()
        opModeStatus = "setup"
        telemetryQueue = 0
        telemetryItems.clear()
        packet = null
    }

    /**
     * Ensure an exception is not thrown due to the telemetry queue being bigger than 255 objects.
     */
    private fun flushTelemetryQueue() {
        telemetryQueue++
        if (telemetryQueue >= 255) {
            // We have to flush out telemetry as the queue is too big
            update()
            if (opMode.telemetry.isAutoClear) {
                // Flush successful
                Dbg.logd("Telemetry queue exceeded 255 messages, auto-pushing to flush...")
            }
        }
    }

    /**
     * Create a new telemetry object and add it to the management queue.
     */
    private fun createTelemetryItem(value: String, retained: Boolean): Item {
        val item = opMode.telemetry.addData(value, "")
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

    @Suppress("KDocMissingDocumentation")
    @Deprecated("Captions are not used with DualTelemetry", replaceWith = ReplaceWith("add(caption + format, args)"))
    override fun addData(caption: String, format: String, vararg args: Any): Item? {
        return add(caption + format, args)
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated("Captions are not used with DualTelemetry", replaceWith = ReplaceWith("add(caption + value)"))
    override fun addData(caption: String, value: Any): Item? {
        return add(caption + value)
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "Captions and function providers are not used with DualTelemetry",
        replaceWith = ReplaceWith("add(caption) // Use polling loop and fstring for provider")
    )
    override fun <T : Any> addData(caption: String, valueProducer: Func<T>): Item? {
        return add(caption, valueProducer.value())
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "Captions and function providers are not used with DualTelemetry",
        replaceWith = ReplaceWith("add(caption + format) // Use polling loop and fstring for provider")
    )
    override fun <T : Any> addData(caption: String, format: String, valueProducer: Func<T>): Item? {
        return add(caption + format, valueProducer.value())
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "removeItem has been migrated to removeRetained with usages in retained telemetry (will still work for non-retained)",
        replaceWith = ReplaceWith("removeRetained(item)")
    )
    override fun removeItem(item: Item): Boolean {
        return removeRetained(item)
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry",
        replaceWith = ReplaceWith("add(format, args)")
    )
    override fun addLine(): Telemetry.Line? {
        add("")
        return null
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry",
        replaceWith = ReplaceWith("add(format, args)")
    )
    override fun addLine(lineCaption: String): Telemetry.Line? {
        add(lineCaption)
        return null
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry",
        replaceWith = ReplaceWith("add(format, args)")
    )
    override fun removeLine(line: Telemetry.Line): Boolean {
        return false
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry",
        replaceWith = ReplaceWith("add(format, args)")
    )
    override fun getItemSeparator(): String {
        return opMode.telemetry.itemSeparator
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry",
        replaceWith = ReplaceWith("add(format, args)")
    )
    override fun setItemSeparator(itemSeparator: String) {
        opMode.telemetry.itemSeparator = itemSeparator
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated("Captions are not used with DualTelemetry and should always be left as an empty string")
    override fun setCaptionValueSeparator(captionValueSeparator: String) {
        opMode.telemetry.captionValueSeparator = captionValueSeparator
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry log should not be accessed with this method, use log(msg) methods directly as FtcDashboard logging will not work with this method.",
        replaceWith = ReplaceWith("log(...)")
    )
    override fun log(): Telemetry.Log {
        return opMode.telemetry.log()
    }
}
