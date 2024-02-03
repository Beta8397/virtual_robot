package org.murraybridgebunyips.bunyipslib;

import java.util.Arrays;
import java.util.IllegalFormatFlagsException;
import java.util.List;
import java.util.Locale;

/**
 * Text and string manipulation utilities.
 *
 * @author Lucas Bubner, 2023
 */
public class Text {
    /**
     * Format a string using only '%' placeholders.
     * Differs from String.format() as type can be omitted.
     * <p>
     * {@code formatString("Hello %!", "world")} -> {@code "Hello world!"}
     */
    public static String formatString(String fstring, List<Object> objs) {
        // Replace all % with the strings in order
        int occurrences = 0;
        StringBuilder newString = new StringBuilder();
        for (int i = 0; i < fstring.length(); i++) {
            if (fstring.charAt(i) == '%') {
                // Remove character and insert new string
                try {
                    newString.append(objs.get(occurrences));
                } catch (IndexOutOfBoundsException e) {
                    throw new IllegalFormatFlagsException("Missing args for '%' formatString() placeholders!");
                }
                occurrences++;
                continue;
            }
            newString.append(fstring.charAt(i));
        }
        if (occurrences != objs.size()) {
            Dbg.warn(getCallingUserCodeFunction(), "Missing '%' placeholders for formatString() objects!");
        }
        return newString.toString();
    }

    public static String formatString(String fstring, Object... objs) {
        return formatString(fstring, Arrays.asList(objs));
    }

    /**
     * Round a number to a certain number of decimal points.
     */
    public static double round(double num, int toDecimalPlaces) {
        // noinspection MalformedFormatString
        return Double.parseDouble(String.format(Locale.getDefault(), "%." + toDecimalPlaces + "f", num));
    }

    /**
     * Round a number to a certain number of decimal points.
     */
    public static float round(float num, int toDecimalPlaces) {
        // noinspection MalformedFormatString
        return Float.parseFloat(String.format(Locale.getDefault(), "%." + toDecimalPlaces + "f", num));
    }

    /**
     * Get the calling function of the current context.
     */
    public static StackTraceElement getCallingUserCodeFunction() {
        StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();
        // Keep going down the stack trace until we leave the BunyipsLib package
        for (StackTraceElement stackTraceElement : stackTrace) {
            if (stackTraceElement.getMethodName().equals("getStackTrace")) continue;
            // If porting, ensure the string below is set to the package name of BunyipsLib
            if (!stackTraceElement.getClassName().startsWith("org.murraybridgebunyips.bunyipslib")) {
                return stackTraceElement;
            }
        }
        // If we can't find the calling function, we'll settle for the first stack trace element
        // This is likely going to be the getStackTrace() function, and we will warn the user as well
        Dbg.warn("Could not find calling function in getCallingUserCodeFunction()!");
        return stackTrace[0];
    }
}
