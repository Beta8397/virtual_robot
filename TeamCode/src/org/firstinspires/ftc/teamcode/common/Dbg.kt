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

import android.util.Log

/**
 * Provide utility methods for debug logging
 */
object Dbg {
    /**
     * Tag used by logcat
     */
    private const val TAG = "BELLOWER"
    private const val ERR_PREPEND = "!! "
    private const val WRN_PREPEND = "! "

    /**
     * Log an error message
     * Messages will be prepended with the ERROR_PREPEND string
     * @param message message to error
     */
    fun error(message: String) {
        Log.e(TAG, ERR_PREPEND + message)
    }

    fun error(format: String, vararg args: Any?) {
        error(String.format(format, *args))
    }

    /**
     * Report out a stacktrace
     * @param e throwable
     */
    fun sendStacktrace(e: Throwable) {
        Log.e(TAG, e.toString())
        for (el in e.stackTrace) {
            Log.e(TAG, el.toString())
        }
    }

    /**
     * Log a warning message
     * @param message message to warn
     */
    fun warn(message: String) {
        Log.w(TAG, WRN_PREPEND + message)
    }

    fun warn(format: String, vararg args: Any?) {
        warn(String.format(format, *args))
    }

    /**
     * Log a debug message
     * @param message message to log
     */
    fun log(message: String) {
        Log.d(TAG, message)
    }

    fun log(format: String, vararg args: Any?) {
        log(String.format(format, *args))
    }
}