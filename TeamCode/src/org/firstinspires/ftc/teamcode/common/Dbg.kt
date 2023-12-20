/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
package org.firstinspires.ftc.teamcode.common

import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.common.Text.formatString

/**
 * Provide utility methods for debug logging
 */
object Dbg {
    /**
     * Tag used by logcat
     */
    const val TAG = "BELLOWER"
    private const val ERR_PREPEND = "!! "
    private const val WRN_PREPEND = "! "

    /**
     * Log an error message.
     * Messages will be prepended with the ERROR_PREPEND string
     * Best used in a scenario where the program cannot continue normally or at required functionality
     * @param message message to error
     */
    @JvmStatic
    fun error(message: String) {
        RobotLog.ee(TAG, ERR_PREPEND + message)
    }

    @JvmStatic
    fun error(format: String, vararg args: Any?) {
        error(formatString(format, *args))
    }

    /**
     * Report out a stacktrace.
     * @param e throwable
     */
    @JvmStatic
    fun sendStacktrace(e: Throwable) {
        RobotLog.ee(TAG, e.toString())
        for (el in e.stackTrace) {
            RobotLog.ee(TAG, el.toString())
        }
    }

    /**
     * Log a warning message.
     * Messages will be prepended with the WRN_PREPEND string
     * Best used in a scenario where the program can continue, but the user should be warned
     * @param message message to warn
     */
    @JvmStatic
    fun warn(message: String) {
        RobotLog.ww(TAG, WRN_PREPEND + message)
    }

    @JvmStatic
    fun warn(format: String, vararg args: Any?) {
        warn(formatString(format, *args))
    }

    /**
     * Log an internal debug message.
     * Best used from critical classes to log internal state
     * @param message message to log
     */
    @JvmStatic
    fun logd(message: String) {
        RobotLog.dd(TAG, message)
    }

    @JvmStatic
    fun logd(format: String, vararg args: Any?) {
        logd(formatString(format, *args))
    }

    /**
     * Log a user message.
     * Best used to log a message or value to Logcat from user code
     * @param message message to log
     */
    @JvmStatic
    fun log(message: String) {
        RobotLog.ii(TAG, message)
    }

    @JvmStatic
    fun log(format: String, vararg args: Any?) {
        log(formatString(format, *args))
    }
}