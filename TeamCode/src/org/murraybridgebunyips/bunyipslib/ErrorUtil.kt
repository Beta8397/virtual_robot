package org.murraybridgebunyips.bunyipslib

import java.io.PrintWriter
import java.io.StringWriter

/**
 * Util to prevent unhandled exceptions from crashing the app.
 * This will log the exception and stacktrace to the Driver Station, allowing OpModes to continue running.
 * In the past, exceptions used to cause an EMERGENCY STOP condition, but has changed to a more modern popup window,
 * however, this class is still useful as it will not terminate the OpMode and will allow code to continue
 * while providing full logging in both Logcat and the Driver Station.
 * @see NullSafety
 */
object ErrorUtil {
    private const val MAX_STACKTRACE_CHARS = 250

    @Throws(InterruptedException::class)
    fun handleCatchAllException(e: Exception, log: (msg: String) -> Unit) {
        if (e is NullPointerException) {
            for (component in NullSafety.unusableComponents) {
                if (e.localizedMessage?.contains(component) == true) {
                    // This error is caused by a null component, which is handled by NullSafety
                    // As such, we can swallow it from appearing on the Driver Station
                    Dbg.warn("Attempted to utilise null-aware unusable object: $component")
                    return
                }
            }
        }
        log("encountered exception! <${e.localizedMessage}>")
        if (e.cause != null) {
            log("caused by: ${e.cause}")
        }
        var stack = stackTraceAsString(e)
        if (stack.length > MAX_STACKTRACE_CHARS) {
            stack = stack.substring(0, MAX_STACKTRACE_CHARS - 4)
            stack += " ..."
        }
        log("stacktrace (max->$MAX_STACKTRACE_CHARS): $stack")
        Dbg.error("Exception caught! Stacktrace:")
        Dbg.sendStacktrace(e)
        if (e is InterruptedException) {
            Dbg.error("Interrupted exception called, raising to superclass...")
            // FTC SDK must handle this
            throw e
        }
        if (e is EmergencyStop) {
            Dbg.error("Emergency stop exception called, raising to superclass...")
            // This is a special exception where we shouldn't continue running the OpMode
            // We will let the FTC SDK handle it in terminating the OpMode and displaying the popup
            throw e
        }
    }

    private fun stackTraceAsString(e: Throwable): String {
        val sw = StringWriter()
        val pw = PrintWriter(sw)
        e.printStackTrace(pw)
        return sw.toString()
    }
}