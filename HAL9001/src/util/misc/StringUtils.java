package util.misc;

import util.exceptions.DumpsterFireException;
import util.exceptions.ExceptionChecker;

import static java.lang.Math.min;

/**
 * A static utility class for performing common operations on Strings.
 * <p>
 * Creation Date: 5/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see String
 * @since 1.1.0
 */
public class StringUtils {

    /**
     * A private constructor for HALFileUtil. Used to make the class static.
     */
    private StringUtils() {
    }

    /**
     * Removes the first character of the given string.
     *
     * @param str The input string.
     * @return The input string without its first character.
     */
    public static String removeFirstChar(String str) {
        return str.substring(1);
    }

    /**
     * Removes the last character of the given string.
     *
     * @param str The input string.
     * @return The input string without its last character.
     */
    public static String removeLastChar(String str) {
        return str.substring(0, str.length() - 1);
    }

    /**
     * Reverses the given string.
     *
     * @param str The input string.
     * @return The reversed version of the input string.
     */
    public static String reverseString(String str) {
        StringBuilder strBuilder = new StringBuilder(str);
        return strBuilder.reverse().toString();
    }

    /**
     * Sets a character in the given string to a specified value.
     *
     * @param str The input string.
     * @param charIdx The index of the character to change.
     * @param c The character to replace at the given index.
     * @return The string with a character replaced at the given index.
     *
     * @throws IndexOutOfBoundsException Throws this exception when the index does not point to a location within the string.
     */
    public static String setChar(String str, int charIdx, char c) {
        ExceptionChecker.assertTrue(charIdx < str.length(), new IndexOutOfBoundsException("Char index must point to a location within the string."));
        StringBuilder strBuilder = new StringBuilder(str);
        strBuilder.setCharAt(charIdx, c);
        return strBuilder.toString();
    }

    /**
     * Strips a character from both ends of a given string, stopping when it reaches a region of the string without the given character.
     *
     * @param str The input string.
     * @param charToStrip The character to strip from both ends of the string.
     * @return The input string with both ends stripped of the given characters.
     */
    public static String bilateralStrip(String str, char charToStrip) {
        int startIdx = 0;
        int endIdx = 0;

        //removes leading chars
        for (int i = 0; i < str.length(); i++) {
            if (str.charAt(i) != charToStrip) {
                startIdx = i;
                break;
            }
        }

        //removes trailing chars
        for(int i = str.length()-1; i >= 0; i--) {
            if(str.charAt(i) != charToStrip) {
                endIdx = i;
                break;
            }
        }

        String strippedString = str.substring(startIdx, endIdx + 1);
        strippedString = strippedString.equals(String.valueOf(charToStrip)) ? "" : strippedString;
        return strippedString;
    }

    /**
     * Repeats a given character a specified number of times.
     *
     * @param c             The input character.
     * @param timesToRepeat The number of times to repeat that character.
     * @return The given character repeated the specified number of times.
     * @throws DumpsterFireException Throws this exception if timesToRepeat is negative.
     */
    public static String repeatCharacter(char c, int timesToRepeat) {
        if (timesToRepeat == 0) return "";

        ExceptionChecker.assertTrue(timesToRepeat > 0, new DumpsterFireException("Cannot repeat a char " + timesToRepeat + " times."));
        return new String(new char[timesToRepeat]).replace('\0', c);
    }

    /**
     * Splits the given string into equal chunks of a given maximum size.
     *
     * @param text         The input string.
     * @param maxChunkSize The maximum size of each chunk.
     * @return An array of chunks, each with a maximum size of maxChunkSize.
     */
    public static String[] splitEqually(String text, int maxChunkSize) {
        ExceptionChecker.assertTrue(maxChunkSize > 0, new DumpsterFireException("Max chunk size for equally splitting a string must be greater than 0."));
        String[] ret = new String[(text.length() + maxChunkSize - 1) / maxChunkSize];
        int i = 0;
        for (int start = 0; start < text.length(); start += maxChunkSize) {
            ret[i] = text.substring(start, min(text.length(), start + maxChunkSize));
            i++;
        }
        return ret;
    }
}
