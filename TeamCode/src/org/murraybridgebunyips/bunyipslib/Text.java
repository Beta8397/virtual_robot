package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.math.BigDecimal;
import java.math.MathContext;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Text and string manipulation utilities.
 *
 * @author Lucas Bubner, 2023
 */
public final class Text {
    private Text() {
    }

    /**
     * Format a string using only '%' placeholders.
     * Differs from String.format() as type can be omitted.
     * <p>
     * {@code formatString("Hello %!", "world")} -> {@code "Hello world!"}
     *
     * @param fstring The string to format
     * @param objs    The objects to insert into the string
     * @return The formatted string
     */
    public static String formatString(@NonNull String fstring, @Nullable List<Object> objs) {
        if (objs == null || objs.isEmpty())
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

    /**
     * Format a string using only '%' placeholders.
     *
     * @param fstring The string to format
     * @param objs    The objects to insert into the string
     * @return The formatted string
     */
    public static String formatString(@NonNull String fstring, @Nullable Object... objs) {
        return formatString(fstring, Arrays.asList(objs));
    }

    /**
     * Round a number to a certain number of decimal points.
     *
     * @param num      The number to round
     * @param thDigits The number of decimal places to use after the decimal point
     * @return The rounded number, or 0 if the number is null
     */
    public static double round(@Nullable Double num, int thDigits) {
        return round(num, thDigits, -1);
    }

    /**
     * Round a number to a certain number of decimal points.
     *
     * @param num      The number to round
     * @param thDigits The number of decimal places to use after the decimal point
     * @return The rounded number, or 0 if the number is null
     */
    public static float round(@Nullable Float num, int thDigits) {
        if (num == null)
            return 0;
        return (float) round(Double.valueOf(num), thDigits, -1);
    }

    /**
     * Round a number to a certain number of decimal points.
     *
     * @param num      The number to round
     * @param thDigits The number of decimal places to use after the decimal point
     * @param sigFigs  The number of significant figures to use
     * @return The rounded number, or 0 if the number is null
     */
    public static float round(@Nullable Float num, int thDigits, int sigFigs) {
        if (num == null)
            return 0;
        return (float) round(Double.valueOf(num), thDigits, sigFigs);
    }

    /**
     * Round a number to a certain number of decimal points.
     *
     * @param num      The number to round
     * @param thDigits The number of decimal places to use after the decimal point
     * @param sigFigs  The number of significant figures to use
     * @return The rounded number, or 0 if the number is null, or 0 if the number is null
     */
    public static double round(@Nullable Double num, int thDigits, int sigFigs) {
        if (num == null)
            return 0;
        if (thDigits == 0)
            return Math.round(num);
        BigDecimal bd = new BigDecimal(Double.toString(num));
        bd = bd.setScale(thDigits, RoundingMode.HALF_UP);
        if (sigFigs != -1)
            bd = bd.round(new MathContext(sigFigs, RoundingMode.HALF_UP));
        return bd.doubleValue();
    }

    /**
     * Get the calling user code function of the current context by looking at the stacktrace until it leaves BunyipsLib.
     */
    public static StackTraceElement getCallingUserCodeFunction() {
        StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();
        // Keep going down the stack trace until we leave the BunyipsLib package
        for (StackTraceElement stackTraceElement : stackTrace) {
            // dalvik.system.VMStack.getThreadStackTrace(Native Method) is not useful, which shows up in the stacktrace
            if (stackTraceElement.toString().toLowerCase().contains("stacktrace")) continue;
            // If porting, ensure the string below is set to the package name of BunyipsLib
            if (!stackTraceElement.getClassName().startsWith("org.murraybridgebunyips.bunyipslib")) {
                return stackTraceElement;
            }
        }
        // If we can't find the calling function, then we can't return a stack trace element
        Dbg.warn("Could not find calling function in getCallingUserCodeFunction()!");
        return new StackTraceElement("Unknown", "userMethod", "User Code", -1);
    }

    /**
     * Begin building an HTML string.
     *
     * @return A HtmlBuilder instance
     */
    public static HtmlBuilder html() {
        return new HtmlBuilder();
    }

    /**
     * Allows for the building of HTML strings, similar to the DualTelemetry HtmlItem for Driver Station telemetry.
     * This serves as a good alternative when not working with the Driver Station telemetry messages, such as in the case
     * of Driver Station logs, FtcDashboard, or other HTML output.
     */
    public static class HtmlBuilder {
        private final StringBuilder html = new StringBuilder();
        private final ArrayList<String> tags = new ArrayList<>();

        /**
         * Insert normal text into the HTML string.
         *
         * @param text The text to insert
         * @return The HtmlBuilder instance
         */
        public HtmlBuilder text(String text) {
            html.append(text);
            return this;
        }

        /**
         * Insert a new line into the HTML string.
         *
         * @return The HtmlBuilder instance
         */
        public HtmlBuilder endl() {
            html.append("\n");
            return this;
        }

        /**
         * Insert bold text into the HTML string.
         *
         * @param text The text to insert
         * @return The HtmlBuilder instance
         */
        public HtmlBuilder bold(String text) {
            html.append("<b>").append(text).append("</b>");
            return this;
        }

        /**
         * Insert italic text into the HTML string.
         *
         * @param text The text to insert
         * @return The HtmlBuilder instance
         */
        public HtmlBuilder italic(String text) {
            html.append("<i>").append(text).append("</i>");
            return this;
        }

        /**
         * Insert big text into the HTML string.
         *
         * @param text The text to insert
         * @return The HtmlBuilder instance
         */
        public HtmlBuilder big(String text) {
            html.append("<big>").append(text).append("</big>");
            return this;
        }

        /**
         * Insert small text into the HTML string.
         *
         * @param text The text to insert
         * @return The HtmlBuilder instance
         */
        public HtmlBuilder small(String text) {
            html.append("<small>").append(text).append("</small>");
            return this;
        }

        /**
         * Insert underline text into the HTML string.
         *
         * @param text The text to insert
         * @return The HtmlBuilder instance
         */
        public HtmlBuilder underline(String text) {
            html.append("<u>").append(text).append("</u>");
            return this;
        }

        /**
         * Insert strikethrough text into the HTML string.
         *
         * @param text The text to insert
         * @return The HtmlBuilder instance
         */
        public HtmlBuilder strikethrough(String text) {
            html.append("<s>").append(text).append("</s>");
            return this;
        }

        /**
         * Insert superscript text into the HTML string.
         *
         * @param text The text to insert
         * @return The HtmlBuilder instance
         */
        public HtmlBuilder superscript(String text) {
            html.append("<sup>").append(text).append("</sup>");
            return this;
        }

        /**
         * Insert subscript text into the HTML string.
         *
         * @param text The text to insert
         * @return The HtmlBuilder instance
         */
        public HtmlBuilder subscript(String text) {
            html.append("<sub>").append(text).append("</sub>");
            return this;
        }

        /**
         * Insert a header into the HTML string.
         *
         * @param text  The text to insert
         * @param level The header level
         * @return The HtmlBuilder instance
         */
        public HtmlBuilder header(int level, String text) {
            if (level < 1 || level > 6) {
                Dbg.warn("Invalid header level " + level + " in HtmlBuilder.header()");
                return this;
            }
            html.append("<h").append(level).append(">").append(text).append("</h").append(level).append(">");
            return this;
        }

        /**
         * Insert a custom tag supplied by the user into the HTML string. Note that these tags are limited to the
         * HTML tags that are available as part of `Html.fromHtml()`.
         *
         * @param tag  The tag to insert, e.g. "div", "span", "p"
         * @param text The text to insert
         * @return The HtmlBuilder instance
         */
        public HtmlBuilder tag(String tag, String text) {
            html.append("<").append(tag).append(">").append(text).append("</").append(tag).append(">");
            return this;
        }

        /**
         * Insert a foreground color to display the text in.
         *
         * @param color The color to use (any valid CSS color)
         * @param text  The text to insert
         * @return The HtmlBuilder instance
         */
        public HtmlBuilder color(String color, String text) {
            html.append("<font color=\"").append(color).append("\">").append(text).append("</font>");
            return this;
        }

        /**
         * Insert a background color to display the text in.
         *
         * @param color The color to use (any valid CSS color)
         * @param text  The text to insert
         * @return The HtmlBuilder instance
         */
        public HtmlBuilder bgColor(String color, String text) {
            html.append("<span style=\"background-color:").append(color).append("\">").append(text).append("</span>");
            return this;
        }

        /**
         * Build the HTML string.
         *
         * @return The HTML string built by the HtmlBuilder
         */
        @NonNull
        @Override
        public String toString() {
            return html.toString();
        }

        @Override
        public boolean equals(Object obj) {
            if (obj == this) return true;
            if (obj.toString().equals(html.toString())) return true;
            if (!(obj instanceof HtmlBuilder)) return false;
            HtmlBuilder other = (HtmlBuilder) obj;
            return html.toString().equals(other.html.toString());
        }

        @Override
        public int hashCode() {
            return html.toString().hashCode();
        }
    }
}
