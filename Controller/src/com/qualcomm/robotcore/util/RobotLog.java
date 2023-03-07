/*
 * Copyright (c) 2014, 2015 Qualcomm Technologies Inc
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Qualcomm Technologies Inc nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Modified by Team Beta for use in the virtual_robot simulator.
 */

package com.qualcomm.robotcore.util;

import android.util.Log;
import java.util.Calendar;

/**
 * Allows consistent logging across all RobotCore packages
 */
@SuppressWarnings("WeakerAccess")
public class RobotLog {

    /*
     * Currently only supports android style logging, but may support more in the future.
     */

    /*
     * Only contains static utility methods
     */
    private RobotLog() {
    }

    //------------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------------

    public static final String OPMODE_START_TAG = "******************** START - OPMODE %s ********************";
    public static final String OPMODE_STOP_TAG = "******************** STOP - OPMODE %s ********************";

    private static double msTimeOffset = 0.0;

    public static final String TAG = "RobotCore";

    /*
     * Prefixing with exec causes the call to destory to kill logcat instead of it's spawning shell.
     */
    private static final String logcatCommandRaw = "logcat";
    private static final String logcatCommand = "exec " + logcatCommandRaw;
    private static final int kbLogcatQuantum = 4 * 1024;
    private static final int logcatRotatedLogsMax = 4;
    private static final String logcatFormat = "threadtime";
    private static final String logcatFilter = "UsbRequestJNI:S UsbRequest:S art:W ThreadPool:W System:W ExtendedExtractor:W OMXClient:W MediaPlayer:W dalvikvm:W  *:V";

    private static Calendar matchStartTime = null;

    //------------------------------------------------------------------------------------------------
    // Time Synchronization
    //------------------------------------------------------------------------------------------------


    //------------------------------------------------------------------------------------------------
    // Logging API
    //------------------------------------------------------------------------------------------------

    public static void a(String format, Object... args) {
        v(String.format(format, args));
    }

    public static void a(String message) {
        internalLog(Log.ASSERT, TAG, message);
    }

    public static void aa(String tag, String format, Object... args) {
        vv(tag, String.format(format, args));
    }

    public static void aa(String tag, String message) {
        internalLog(Log.ASSERT, tag, message);
    }

    public static void aa(String tag, Throwable throwable, String format, Object... args) {
        vv(tag, throwable, String.format(format, args));
    }

    public static void aa(String tag, Throwable throwable, String message) {
        internalLog(Log.ASSERT, tag, throwable, message);
    }

    public static void v(String format, Object... args) {
        v(String.format(format, args));
    }

    public static void v(String message) {
        internalLog(Log.VERBOSE, TAG, message);
    }

    public static void vv(String tag, String format, Object... args) {
        vv(tag, String.format(format, args));
    }

    public static void vv(String tag, String message) {
        internalLog(Log.VERBOSE, tag, message);
    }

    public static void vv(String tag, Throwable throwable, String format, Object... args) {
        vv(tag, throwable, String.format(format, args));
    }

    public static void vv(String tag, Throwable throwable, String message) {
        internalLog(Log.VERBOSE, tag, throwable, message);
    }

    public static void d(String format, Object... args) {
        d(String.format(format, args));
    }

    public static void d(String message) {
        internalLog(Log.DEBUG, TAG, message);
    }

    public static void dd(String tag, String format, Object... args) {
        dd(tag, String.format(format, args));
    }

    public static void dd(String tag, String message) {
        internalLog(Log.DEBUG, tag, message);
    }

    public static void dd(String tag, Throwable throwable, String format, Object... args) {
        dd(tag, throwable, String.format(format, args));
    }

    public static void dd(String tag, Throwable throwable, String message) {
        internalLog(Log.DEBUG, tag, throwable, message);
    }

    public static void i(String format, Object... args) {
        i(String.format(format, args));
    }

    public static void i(String message) {
        internalLog(Log.INFO, TAG, message);
    }

    public static void ii(String tag, String format, Object... args) {
        ii(tag, String.format(format, args));
    }

    public static void ii(String tag, String message) {
        internalLog(Log.INFO, tag, message);
    }

    public static void ii(String tag, Throwable throwable, String format, Object... args) {
        ii(tag, throwable, String.format(format, args));
    }

    public static void ii(String tag, Throwable throwable, String message) {
        internalLog(Log.INFO, tag, throwable, message);
    }

    public static void w(String format, Object... args) {
        w(String.format(format, args));
    }

    public static void w(String message) {
        internalLog(Log.WARN, TAG, message);
    }

    public static void ww(String tag, String format, Object... args) {
        ww(tag, String.format(format, args));
    }

    public static void ww(String tag, String message) {
        internalLog(Log.WARN, tag, message);
    }

    public static void ww(String tag, Throwable throwable, String format, Object... args) {
        ww(tag, throwable, String.format(format, args));
    }

    public static void ww(String tag, Throwable throwable, String message) {
        internalLog(Log.WARN, tag, throwable, message);
    }

    public static void e(String format, Object... args) {
        e(String.format(format, args));
    }

    public static void e(String message) {
        internalLog(Log.ERROR, TAG, message);
    }

    public static void ee(String tag, String format, Object... args) {
        ee(tag, String.format(format, args));
    }

    public static void ee(String tag, String message) {
        internalLog(Log.ERROR, tag, message);
    }

    public static void ee(String tag, Throwable throwable, String format, Object... args) {
        ee(tag, throwable, String.format(format, args));
    }

    public static void ee(String tag, Throwable throwable, String message) {
        internalLog(Log.ERROR, tag, throwable, message);
    }

    public static void internalLog(int priority, String tag, String message) {
//        if (msTimeOffset == 0) {
//            System.out.println(priority + " " + tag + message);
//        }
    }

    public static void internalLog(int priority, String tag, Throwable throwable, String message) {
//        internalLog(priority, tag, message);
//        logStackTrace(tag, throwable);
    }

    public static void logExceptionHeader(Exception e, String format, Object... args) {
        String message = String.format(format, args);
        RobotLog.e("exception %s(%s): %s [%s]", e.getClass().getSimpleName(), e.getMessage(), message, getStackTop(e));
    }

    public static void logExceptionHeader(String tag, Exception e, String format, Object... args) {
        String message = String.format(format, args);
        RobotLog.ee(tag, "exception %s(%s): %s [%s]", e.getClass().getSimpleName(), e.getMessage(), message, getStackTop(e));
    }

    private static StackTraceElement getStackTop(Exception e) {
        StackTraceElement[] frames = e.getStackTrace();
        return frames.length > 0 ? frames[0] : null;
    }

    /**
     * @deprecated obsolete capitalization
     */
    @Deprecated
    public static void logStacktrace(Throwable e) {
        logStackTrace(e);
    }

    public static void logStackTrace(Throwable e) {
        logStackTrace(TAG, e);
    }

    public static void logStackTrace(Thread thread, String format, Object... args) {
    }

    public static void logStackTrace(Thread thread, StackTraceElement[] stackTrace) {
    }

    public static void logStackTrace(String tag, Throwable e) {

    }

    private static void logStackFrames(StackTraceElement[] stackTrace) {
        for (StackTraceElement frame : stackTrace) {
            RobotLog.e("    at %s", frame.toString());
        }
    }

    public static void logAndThrow(String errMsg) {

    }


    public static void logAppInfo() {
        RobotLog.i("LogAppInfo: virtual_robot");
    }

    public static void logDeviceInfo() {
        RobotLog.i("logDeviceInfo: virtual_robot");
    }

    public static void logBytes(String tag, String caption, byte[] data, int cb) {
        logBytes(tag, caption, data, 0, cb);
    }

    public static void logBytes(String tag, String caption, byte[] data, int ibStart, int cb) {
        int cbLine = 16;
        char separator = ':';
        for (int ibFirst = ibStart; ibFirst < cb; ibFirst += cbLine) {
            StringBuilder line = new StringBuilder();
            for (int i = 0; i < cbLine; i++) {
                int ib = i + ibFirst;
                if (ib >= cb)
                    break;
                line.append(String.format("%02x ", data[ib]));
            }
            vv(tag, "%s%c %s", caption, separator, line.toString());
            separator = '|';
        }
    }
    public static boolean setGlobalErrorMsg(String message) {return true;}

    public static void setGlobalErrorMsg(String format, Object... args) {}


}