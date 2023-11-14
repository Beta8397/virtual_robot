package org.firstinspires.ftc.teamcode.common

import java.io.PrintWriter
import java.io.StringWriter

/**
 * Util to prevent unhandled exceptions from crashing the app
 */
object ErrorUtil {
    private const val MAX_STACKTRACE_CHARS = 250

    @Throws(InterruptedException::class)
    fun handleCatchAllException(e: Throwable, log: (msg: String) -> Unit) {
        if (e is NullPointerException) {
            for (component in NullSafety.unusableComponents) {
                if (e.message?.contains(component) == true) {
                    // This error is caused by a null component, which is handled by NullSafety
                    // As such, we can swallow it from appearing on the Driver Station
                    Dbg.warn("Attempted to utilise null-aware unusable object: $component")
                    return
                }
            }
        }
        log("encountered exception! <${e.message}>")
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
            // FTC SDK must handle this
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