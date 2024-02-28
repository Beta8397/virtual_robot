package org.murraybridgebunyips.bunyipslib;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * Text and string manipulation utilities.
 *
 * @author Lucas Bubner, 2023
 */
public class Text {
    private Text() {
    }

    /**
     * Format a string using only '%' placeholders.
     * Differs from String.format() as type can be omitted.
     * <p>
     * {@code formatString("Hello %!", "world")} -> {@code "Hello world!"}
     */
    public static String formatString(String fstring, List<Object> objs) {
        if (objs.isEmpty())
            return fstring;
        // Replace all % with the strings in order
        int occurrences = 0;
        StringBuilder newString = new StringBuilder();
        for (int i = 0; i < fstring.length(); i++) {
            if (fstring.charAt(i) == '%') {
                // Remove character and insert new string
                try {
                    // Check for \ before %, if so, add a % to the string instead of the value
                    if (i > 0 && fstring.charAt(i - 1) == '\\') {
                        newString.append("%");
                        // Remove the \ from the string, as it is no longer needed
                        newString.deleteCharAt(newString.length() - 2);
                        // Skip occurrence incrementation
                        continue;
                    } else {
                        newString.append(objs.get(occurrences));
                    }
                } catch (IndexOutOfBoundsException e) {
                    // User did not provide enough arguments, we'll just append a % and continue
                    newString.append("%");
                }
                occurrences++;
                continue;
            }
            newString.append(fstring.charAt(i));
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
        if (toDecimalPlaces == 0) {
            return Math.round(num);
        }
        // noinspection MalformedFormatString
        return Double.parseDouble(String.format(Locale.getDefault(), "%." + toDecimalPlaces + "f", num));
    }

    /**
     * Round a number to a certain number of decimal points.
     */
    public static float round(float num, int toDecimalPlaces) {
        return (float) round((double) num, toDecimalPlaces);
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
