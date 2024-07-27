package org.murraybridgebunyips.bunyipslib

import com.acmerobotics.dashboard.config.Config
import java.io.PrintWriter
import java.io.StringWriter
import java.util.function.Consumer

/**
 * OpMode "userspace" util to prevent user-code unhandled exceptions from crashing the app.
 *
 * This will log the exception and stacktrace to the Driver Station, allowing OpModes to continue running.
 * In the past, exceptions used to cause an EMERGENCY STOP condition, but has changed to a more modern popup window,
 * however, this class is still useful as it will not terminate the OpMode and will allow code to continue
 * while providing full logging in both Logcat and the Driver Station.
 *
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
     *
     * @param e The exception to handle.
     * @param stderr The function to print logging to the Driver Station or other stderr outputs that support HTML parsing.
     * @see runUserMethod for single method executions of exception-prone user code
     */
    @JvmStatic
    fun handle(e: Exception, stderr: Consumer<String>) {
        Dbg.error("Exception caught! Stacktrace:")
        Dbg.sendStacktrace(e)
        val sw = StringWriter()
        e.printStackTrace(PrintWriter(sw))
        var stack = sw.toString()
        if (e is NullPointerException) {
            for (component in Storage.memory().unusableComponents) {
                if (stack.contains(component)) {
                    // This error is caused by a null component, which is handled by NullSafety
                    // As such, we can swallow it from appearing on the Driver Station
                    // Logcat will still receive this log, so we can just early exit.
                    return
                }
            }
        }
        stderr.accept("<font color='red'><b>exception caught! &lt;${e.localizedMessage}&gt;</b></font>")
        if (e.cause != null) {
            stderr.accept("caused by: ${e.cause}")
        }
        if (stack.length > MAX_DS_STACKTRACE_CHARS) {
            stack = stack.substring(0, MAX_DS_STACKTRACE_CHARS - 4)
            stack += " ..."
        }
        stderr.accept("<small>$stack</small>")
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

    /**
     * Handle and run a single user-controlled method by wrapping the execution with an `Exceptions.handle()`.
     *
     * This ensures integrity in methods that are written by a user to be more graceful in execution,
     * ensuring code execution is not stopped on a non-critical exception.
     */
    @JvmStatic
    fun runUserMethod(method: Runnable, opMode: BunyipsOpMode) {
        try {
            method.run()
        } catch (e: Exception) {
            handle(e, opMode.t::log)
        }
    }
}