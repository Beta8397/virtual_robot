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
     * Return a wrapper of {@link StringBuilder} that internally calls {@link #formatString} on all append calls.
     *
     * @return modified StringBuilder with default initial capacity 16
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Return a wrapper of {@link StringBuilder} that internally calls {@link #formatString} on all append calls.
     *
     * @param capacity the initial capacity of the StringBuilder
     * @return modified StringBuilder with initial capacity defined
     */
    public static Builder builder(int capacity) {
        return new Builder(capacity);
    }

    /**
     * Return a wrapper of {@link StringBuilder} that internally calls {@link #formatString} on all append calls.
     *
     * @param initial the initial string of the StringBuilder
     * @return modified StringBuilder with initial string defined
     */
    public static Builder builder(String initial) {
        return new Builder(initial);
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

    /**
     * A modified wrapper of {@link StringBuilder} that internally calls {@link #formatString}  on all append calls.
     */
    public static class Builder implements CharSequence {
        private final StringBuilder i;

        /**
         * Create a new instance of the builder with default initial capacity 16.
         */
        public Builder() {
            i = new StringBuilder();
        }

        /**
         * Create a new instance of the builder with the specified initial string.
         *
         * @param initial the initial string of the StringBuilder
         */
        public Builder(String initial) {
            i = new StringBuilder(initial);
        }

        /**
         * Create a new instance of the builder with the specified initial capacity.
         *
         * @param capacity the initial capacity of the StringBuilder
         */
        public Builder(int capacity) {
            i = new StringBuilder(capacity);
        }

        /**
         * Append an object to the builder.
         *
         * @param o the object to append
         * @return the builder instance
         */
        public Builder append(@Nullable Object o) {
            i.append(o);
            return this;
        }

        /**
         * Append a string to the builder using a {@link #formatString} call.
         *
         * @param fstring the format string
         * @param objs    the objects to insert into the string
         * @return the builder instance
         */
        public Builder append(@NonNull String fstring, @Nullable Object... objs) {
            i.append(formatString(fstring, objs));
            return this;
        }

        /**
         * Appends the string representation of the {@code codePoint}
         * argument to this sequence.
         *
         * <p> The argument is appended to the contents of this sequence.
         * The length of this sequence increases by
         * {@link Character#charCount(int) Character.charCount(codePoint)}.
         *
         * <p> The overall effect is exactly as if the argument were
         * converted to a {@code char} array by the method
         * {@link Character#toChars(int)} and the character in that array
         * were then {@link #append appended} to this character
         * sequence.
         *
         * @param codePoint a Unicode code point
         * @return a reference to this object.
         * @throws IllegalArgumentException if the specified
         *                                  {@code codePoint} isn't a valid Unicode code point
         */
        public Builder appendCodePoint(int codePoint) {
            i.appendCodePoint(codePoint);
            return this;
        }

        /**
         * Removes the characters in a substring of this sequence.
         * The substring begins at the specified {@code start} and extends to
         * the character at index {@code end - 1} or to the end of the
         * sequence if no such character exists. If
         * {@code start} is equal to {@code end}, no changes are made.
         *
         * @param start The beginning index, inclusive.
         * @param end   The ending index, exclusive.
         * @return This object.
         * @throws StringIndexOutOfBoundsException if {@code start}
         *                                         is negative, greater than {@code length()}, or
         *                                         greater than {@code end}.
         */
        public Builder delete(int start, int end) {
            i.delete(start, end);
            return this;
        }

        /**
         * Removes the {@code char} at the specified position in this
         * sequence. This sequence is shortened by one {@code char}.
         *
         * <p>Note: If the character at the given index is a supplementary
         * character, this method does not remove the entire character. If
         * correct handling of supplementary characters is required,
         * determine the number of {@code char}s to remove by calling
         * {@code Character.charCount(thisSequence.codePointAt(index))},
         * where {@code thisSequence} is this sequence.
         *
         * @param index Index of {@code char} to remove
         * @return This object.
         * @throws StringIndexOutOfBoundsException if the {@code index}
         *                                         is negative or greater than or equal to
         *                                         {@code length()}.
         */
        public Builder deleteCharAt(int index) {
            i.deleteCharAt(index);
            return this;
        }

        /**
         * Replaces the characters in a substring of this sequence
         * with characters in the specified {@code String}. The substring
         * begins at the specified {@code start} and extends to the character
         * at index {@code end - 1} or to the end of the
         * sequence if no such character exists. First the
         * characters in the substring are removed and then the specified
         * {@code String} is inserted at {@code start}. (This
         * sequence will be lengthened to accommodate the
         * specified String if necessary.)
         *
         * @param start The beginning index, inclusive.
         * @param end   The ending index, exclusive.
         * @param str   String that will replace previous contents.
         * @return This object.
         * @throws StringIndexOutOfBoundsException if {@code start}
         *                                         is negative, greater than {@code length()}, or
         *                                         greater than {@code end}.
         */
        public Builder replace(int start, int end, String str) {
            i.replace(start, end, str);
            return this;
        }

        /**
         * Inserts the string representation of a subarray of the {@code str}
         * array argument into this sequence. The subarray begins at the
         * specified {@code offset} and extends {@code len} {@code char}s.
         * The characters of the subarray are inserted into this sequence at
         * the position indicated by {@code index}. The length of this
         * sequence increases by {@code len} {@code char}s.
         *
         * @param index  position at which to insert subarray.
         * @param str    A {@code char} array.
         * @param offset the index of the first {@code char} in subarray to
         *               be inserted.
         * @param len    the number of {@code char}s in the subarray to
         *               be inserted.
         * @return This object
         * @throws StringIndexOutOfBoundsException if {@code index}
         *                                         is negative or greater than {@code length()}, or
         *                                         {@code offset} or {@code len} are negative, or
         *                                         {@code (offset+len)} is greater than
         *                                         {@code str.length}.
         */
        public Builder insert(int index, char[] str, int offset, int len) {
            i.insert(index, str, offset, len);
            return this;
        }

        /**
         * Inserts the string representation of the {@code Object}
         * argument into this character sequence.
         * <p>
         * The overall effect is exactly as if the second argument were
         * converted to a string by the method {@link String#valueOf(Object)},
         * and the characters of that string were then
         * {@link #insert inserted} into this character
         * sequence at the indicated offset.
         * <p>
         * The {@code offset} argument must be greater than or equal to
         * {@code 0}, and less than or equal to the {@linkplain #length() length}
         * of this sequence.
         *
         * @param offset the offset.
         * @param obj    an {@code Object}.
         * @return a reference to this object.
         * @throws StringIndexOutOfBoundsException if the offset is invalid.
         */
        public Builder insert(int offset, Object obj) {
            i.insert(offset, obj);
            return this;
        }

        /**
         * Inserts the string representation of the {@code Object}
         * argument into this character sequence after applying a {@link #formatString} call.
         * <p>
         * The overall effect is exactly as if the string was formatted using {@link #formatString},
         * passed into the {@link #insert} method, and then inserted into this character sequence at the indicated offset.
         * <p>
         * The {@code offset} argument must be greater than or equal to
         * {@code 0}, and less than or equal to the {@linkplain #length() length}
         * of this sequence.
         *
         * @param offset  the offset.
         * @param fstring the format string.
         * @param objs    the objects to insert into the string.
         * @return a reference to this object.
         * @throws StringIndexOutOfBoundsException if the offset is invalid.
         */
        public Builder insert(int offset, @NonNull String fstring, @Nullable Object... objs) {
            i.insert(offset, formatString(fstring, objs));
            return this;
        }

        /**
         * Returns the index within this string of the first occurrence of the
         * specified substring. The integer returned is the smallest value
         * <i>k</i> such that:
         * <pre>{@code
         * this.toString().startsWith(str, <i>k</i>)
         * }</pre>
         * is {@code true}.
         *
         * @param str any string.
         * @return if the string argument occurs as a substring within this
         * object, then the index of the first character of the first
         * such substring is returned; if it does not occur as a
         * substring, {@code -1} is returned.
         */
        public int indexOf(String str) {
            return i.indexOf(str);
        }

        /**
         * Returns the index within this string of the first occurrence of the
         * specified substring, starting at the specified index.  The integer
         * returned is the smallest value {@code k} for which:
         * <pre>{@code
         *     k >= Math.min(fromIndex, this.length()) &&
         *                   this.toString().startsWith(str, k)
         * }</pre>
         * If no such value of <i>k</i> exists, then -1 is returned.
         *
         * @param str       the substring for which to search.
         * @param fromIndex the index from which to start the search.
         * @return the index within this string of the first occurrence of the
         * specified substring, starting at the specified index.
         */
        public int indexOf(String str, int fromIndex) {
            return i.indexOf(str, fromIndex);
        }

        /**
         * Returns the index within this string of the rightmost occurrence
         * of the specified substring.  The rightmost empty string "" is
         * considered to occur at the index value {@code this.length()}.
         * The returned index is the largest value <i>k</i> such that
         * <pre>{@code
         * this.toString().startsWith(str, k)
         * }</pre>
         * is true.
         *
         * @param str the substring to search for.
         * @return if the string argument occurs one or more times as a substring
         * within this object, then the index of the first character of
         * the last such substring is returned. If it does not occur as
         * a substring, {@code -1} is returned.
         */
        public int lastIndexOf(String str) {
            return i.lastIndexOf(str);
        }

        /**
         * Returns the index within this string of the last occurrence of the
         * specified substring. The integer returned is the largest value <i>k</i>
         * such that:
         * <pre>{@code
         *     k <= Math.min(fromIndex, this.length()) &&
         *                   this.toString().startsWith(str, k)
         * }</pre>
         * If no such value of <i>k</i> exists, then -1 is returned.
         *
         * @param str       the substring to search for.
         * @param fromIndex the index to start the search from.
         * @return the index within this sequence of the last occurrence of the
         * specified substring.
         */
        public int lastIndexOf(String str, int fromIndex) {
            return i.lastIndexOf(str, fromIndex);
        }

        /**
         * Causes this character sequence to be replaced by the reverse of
         * the sequence. If there are any surrogate pairs included in the
         * sequence, these are treated as single characters for the
         * reverse operation. Thus, the order of the high-low surrogates
         * is never reversed.
         * <p>
         * Let <i>n</i> be the character length of this character sequence
         * (not the length in {@code char} values) just prior to
         * execution of the {@code reverse} method. Then the
         * character at index <i>k</i> in the new character sequence is
         * equal to the character at index <i>n-k-1</i> in the old
         * character sequence.
         *
         * <p>Note that the reverse operation may result in producing
         * surrogate pairs that were unpaired low-surrogates and
         * high-surrogates before the operation. For example, reversing
         * "\uDC00\uD800" produces "\uD800\uDC00" which is
         * a valid surrogate pair.
         *
         * @return a reference to this object.
         */
        public Builder reverse() {
            i.reverse();
            return this;
        }

        @Override
        public int length() {
            return i.length();
        }

        @Override
        public char charAt(int index) {
            return i.charAt(index);
        }

        @NonNull
        @Override
        public CharSequence subSequence(int start, int end) {
            return i.subSequence(start, end);
        }

        @NonNull
        @Override
        public String toString() {
            return i.toString();
        }
    }
}
