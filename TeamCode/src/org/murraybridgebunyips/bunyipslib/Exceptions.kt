package org.murraybridgebunyips.bunyipslib

import com.acmerobotics.dashboard.config.Config
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
@Config
object Exceptions {
    /**
     * Maximum number of characters to display in the stacktrace on the Driver Station.
     */
    @JvmField
    var MAX_DS_STACKTRACE_CHARS = 250

    /**
     * Handle an exception, logging it to the Driver Station and Logcat.
     * @param e The exception to handle.
     * @param stderr The function to print logging to the Driver Station or other stderr outputs that support HTML parsing.
     */
    @Throws(InterruptedException::class)
    fun handle(e: Exception, stderr: (msg: String) -> Unit) {
        if (e is NullPointerException) {
            for (component in Storage.memory().unusableComponents) {
                if (e.localizedMessage?.contains(component) == true) {
                    // This error is caused by a null component, which is handled by NullSafety
                    // As such, we can swallow it from appearing on the Driver Station
                    Dbg.warn("Attempted to utilise null-aware unusable object: $component")
                    return
                }
            }
        }
        stderr("<font color='red'><b>exception caught! &lt;${e.localizedMessage}&gt;</b></font>")
        if (e.cause != null) {
            stderr("caused by: ${e.cause}")
        }
        val sw = StringWriter()
        e.printStackTrace(PrintWriter(sw))
        var stack = sw.toString()
        if (stack.length > MAX_DS_STACKTRACE_CHARS) {
            stack = stack.substring(0, MAX_DS_STACKTRACE_CHARS - 4)
            stack += " ..."
        }
        stderr("<small>$stack</small>")
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
}