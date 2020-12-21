package util.math;

import util.exceptions.ExceptionChecker;
import util.exceptions.HALMathException;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/**
 * A class for doing mathematical operations on arrays. Basically numpy, but less good.
 * <p>
 * Creation Date: 10/19/20.
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @since 1.0.0
 */
public class FakeNumpy {

    /**
     * Private default constructor to make class static.
     */
    private FakeNumpy() {}

    /**
     * Finds the maximum value of an array.
     *
     * @param array The input array.
     * @param <T> The element type of the array.
     * @return The maximum of the array.
     */
    public static <T extends Comparable<? super T>> T max(T[] array) {
        if (array.length == 0) return null;

        return max(array, array.length);
    }

    /**
     * Finds the maximum value of an array.
     *
     * @param array The input array.
     * @param n The maximum index of the elements in the array that will be compared.
     * @param <T> The element type of the array.
     * @return The maximum of the array.
     */
    private static <T extends Comparable<? super T>> T max(T[] array, int n) {
        if (n == 1) return array[0];

        T currentMax = max(array, n - 1);
        return array[n - 1].compareTo(currentMax) > 0 ? array[n - 1] : currentMax;
    }

    /**
     * Finds the maximum value of an array of doubles.
     *
     * @param array The input array.
     * @return The maximum of the array.
     */
    public static double max(double[] array) {
        if (array.length == 0) return 0;

        return max(array, array.length);
    }

    /**
     * Finds the maximum value of an array of doubles.
     *
     * @param array The input array.
     * @param n The maximum index of the elements in the array that will be compared.
     * @return The maximum of the array.
     */
    private static double max(double[] array, int n) {
        if (n == 1) return array[0];

        return Math.max(array[n - 1], max(array, n - 1));
    }

    /**
     * Finds the maximum value of an array of integers.
     *
     * @param array The input array.
     * @return The maximum of the array.
     */
    public static int max(int[] array) {
        if (array.length == 0) return 0;

        return max(array, array.length);
    }

    /**
     * Finds the maximum value of an array of integers.
     *
     * @param array The input array.
     * @param n The maximum index of the elements in the array that will be compared.
     * @return The maximum of the array.
     */
    private static int max(int[] array, int n) {
        if (n == 1) return array[0];

        return Math.max(array[n - 1], max(array, n - 1));
    }

    /**
     * Finds the maximum value of an array of floats.
     *
     * @param array The input array.
     * @return The maximum of the array.
     */
    public static float max(float[] array) {
        if (array.length == 0) return 0;

        return max(array, array.length);
    }

    /**
     * Finds the maximum value of an array of floats.
     *
     * @param array The input array.
     * @param n The maximum index of the elements in the array that will be compared.
     * @return The maximum of the array.
     */
    private static float max(float[] array, int n) {
        if (n == 1) return array[0];

        return Math.max(array[n - 1], max(array, n - 1));
    }

    /**
     * Finds the maximum value of an array of longs.
     *
     * @param array The input array.
     * @return The maximum of the array.
     */
    public static long max(long[] array) {
        if (array.length == 0) return 0;

        return max(array, array.length);
    }

    /**
     * Finds the maximum value of an array of longs.
     *
     * @param array The input array.
     * @param n The maximum index of the elements in the array that will be compared.
     * @return The maximum of the array.
     */
    private static long max(long[] array, int n) {
        if (n == 1) return array[0];

        return Math.max(array[n - 1], max(array, n - 1));
    }

    /**
     * Finds the maximum value of an array of shorts.
     *
     * @param array The input array.
     * @return The maximum of the array.
     */
    public static short max(short[] array) {
        if (array.length == 0) return 0;

        return max(array, array.length);
    }

    /**
     * Finds the maximum value of an array of shorts.
     *
     * @param array The input array.
     * @param n The maximum index of the elements in the array that will be compared.
     * @return The maximum of the array.
     */
    private static short max(short[] array, int n) {
        if (n == 1) return array[0];

        return (short) Math.max(array[n - 1], max(array, n - 1));
    }

    /**
     * Finds the minimum value of an array.
     *
     * @param array The input array.
     * @param <T> The element type of the array.
     * @return The minimum of the array.
     */
    public static <T extends Comparable<? super T>> T min(T[] array) {
        if (array.length == 0) return null;

        return min(array, array.length);
    }

    /**
     * Finds the minimum value of an array.
     *
     * @param array The input array.
     * @param n The maximum index of the elements in the array that will be compared.
     * @param <T> The element type of the array.
     * @return The minimum of the array.
     */
    private static <T extends Comparable<? super T>> T min(T[] array, int n) {
        if (n == 1) return array[0];

        T currentMin = min(array, n - 1);
        return array[n - 1].compareTo(currentMin) < 0 ? array[n - 1] : currentMin;
    }

    /**
     * Finds the minimum value of an array of doubles.
     *
     * @param array The input array.
     * @return The minimum of the array.
     */
    public static double min(double[] array) {
        if (array.length == 0) return 0;

        return min(array, array.length);
    }

    /**
     * Finds the minimum value of an array of doubles.
     *
     * @param array The input array.
     * @param n The maximum index of the elements in the array that will be compared.
     * @return The minimum of the array.
     */
    private static double min(double[] array, int n) {
        if (n == 1) return array[0];

        return Math.min(array[n - 1], min(array, n - 1));
    }

    /**
     * Finds the minimum value of an array of integers.
     *
     * @param array The input array.
     * @return The minimum of the array.
     */
    public static int min(int[] array) {
        if (array.length == 0) return 0;

        return min(array, array.length);
    }

    /**
     * Finds the minimum value of an array of integers.
     *
     * @param array The input array.
     * @param n The maximum index of the elements in the array that will be compared.
     * @return The minimum of the array.
     */
    private static int min(int[] array, int n) {
        if (n == 1) return array[0];

        return Math.min(array[n - 1], min(array, n - 1));
    }

    /**
     * Finds the minimum value of an array of floats.
     *
     * @param array The input array.
     * @return The minimum of the array.
     */
    public static float min(float[] array) {
        if (array.length == 0) return 0;

        return min(array, array.length);
    }

    /**
     * Finds the minimum value of an array of floats.
     *
     * @param array The input array.
     * @param n The maximum index of the elements in the array that will be compared.
     * @return The minimum of the array.
     */
    private static float min(float[] array, int n) {
        if (n == 1) return array[0];

        return Math.min(array[n - 1], min(array, n - 1));
    }

    /**
     * Finds the minimum value of an array of longs.
     *
     * @param array The input array.
     * @return The minimum of the array.
     */
    public static long min(long[] array) {
        if (array.length == 0) return 0;

        return min(array, array.length);
    }

    /**
     * Finds the minimum value of an array of longs.
     *
     * @param array The input array.
     * @param n The maximum index of the elements in the array that will be compared.
     * @return The minimum of the array.
     */
    private static long min(long[] array, int n) {
        if (n == 1) return array[0];

        return Math.min(array[n - 1], min(array, n - 1));
    }

    /**
     * Finds the minimum value of an array of shorts.
     *
     * @param array The input array.
     * @return The minimum of the array.
     */
    public static short min(short[] array) {
        if (array.length == 0) return 0;

        return min(array, array.length);
    }

    /**
     * Finds the minimum value of an array of shorts.
     *
     * @param array The input array.
     * @param n The maximum index of the elements in the array that will be compared.
     * @return The minimum of the array.
     */
    private static short min(short[] array, int n) {
        if (n == 1) return array[0];

        return (short) Math.min(array[n - 1], min(array, n - 1));
    }

    /**
     * Slices an array from startIdx to endIdx (inclusive).
     *
     * @param array The input array.
     * @param startIdx The starting index for the slice.
     * @param endIdx The ending index for the slice.
     * @param <T> The element type of the array.
     * @return The slice of the array that goes from startIdx to endIdx (inclusive).
     */
    public static <T> T[] slice(T[] array, int startIdx, int endIdx) {
        return Arrays.copyOfRange(array, startIdx, endIdx + 1);
    }

    /**
     * Slices a double array from startIdx to endIdx (inclusive).
     *
     * @param array The input array.
     * @param startIdx The starting index for the slice.
     * @param endIdx The ending index for the slice.
     * @return The slice of the array that goes from startIdx to endIdx (inclusive).
     */
    public static double[] slice(double[] array, int startIdx, int endIdx) {
        return Arrays.copyOfRange(array, startIdx, endIdx + 1);
    }

    /**
     * Slices an integer array from startIdx to endIdx (inclusive).
     *
     * @param array The input array.
     * @param startIdx The starting index for the slice.
     * @param endIdx The ending index for the slice.
     * @return The slice of the array that goes from startIdx to endIdx (inclusive).
     */
    public static int[] slice(int[] array, int startIdx, int endIdx) {
        return Arrays.copyOfRange(array, startIdx, endIdx + 1);
    }

    /**
     * Slices a float array from startIdx to endIdx (inclusive).
     *
     * @param array The input array.
     * @param startIdx The starting index for the slice.
     * @param endIdx The ending index for the slice.
     * @return The slice of the array that goes from startIdx to endIdx (inclusive).
     */
    public static float[] slice(float[] array, int startIdx, int endIdx) {
        return Arrays.copyOfRange(array, startIdx, endIdx + 1);
    }

    /**
     * Slices a long array from startIdx to endIdx (inclusive).
     *
     * @param array The input array.
     * @param startIdx The starting index for the slice.
     * @param endIdx The ending index for the slice.
     * @return The slice of the array that goes from startIdx to endIdx (inclusive).
     */
    public static long[] slice(long[] array, int startIdx, int endIdx) {
        return Arrays.copyOfRange(array, startIdx, endIdx + 1);
    }

    /**
     * Slices a short array from startIdx to endIdx (inclusive).
     *
     * @param array The input array.
     * @param startIdx The starting index for the slice.
     * @param endIdx The ending index for the slice.
     * @return The slice of the array that goes from startIdx to endIdx (inclusive).
     */
    public static short[] slice(short[] array, int startIdx, int endIdx) {
        return Arrays.copyOfRange(array, startIdx, endIdx + 1);
    }

    /**
     * Multiplies every element in a double array by a constant double.
     *
     * @param array      The input array.
     * @param multiplier The constant to multiply by.
     * @return The input array times the multiplier.
     */
    public static double[] multiply(double[] array, double multiplier) {
        for (int i = 0; i < array.length; i++) {
            array[i] = array[i] * multiplier;
        }
        return array;
    }

    /**
     * Multiplies every element in an integer array by a constant integer.
     *
     * @param array      The input array.
     * @param multiplier The constant to multiply by.
     * @return The input array times the multiplier.
     */
    public static int[] multiply(int[] array, int multiplier) {
        for (int i = 0; i < array.length; i++) {
            array[i] = array[i] * multiplier;
        }
        return array;
    }

    /**
     * Multiplies every element in a float array by a constant float.
     *
     * @param array      The input array.
     * @param multiplier The constant to multiply by.
     * @return The input array times the multiplier.
     */
    public static float[] multiply(float[] array, float multiplier) {
        for (int i = 0; i < array.length; i++) {
            array[i] = array[i] * multiplier;
        }
        return array;
    }

    /**
     * Multiplies every element in a long array by a constant long.
     *
     * @param array      The input array.
     * @param multiplier The constant to multiply by.
     * @return The input array times the multiplier.
     */
    public static long[] multiply(long[] array, long multiplier) {
        for (int i = 0; i < array.length; i++) {
            array[i] = array[i] * multiplier;
        }
        return array;
    }

    /**
     * Multiplies every element in an integer array by a constant double and rounds it to an integer.
     *
     * @param array      The input array.
     * @param multiplier The constant to multiply by.
     * @return The input array times the multiplier.
     */
    public static int[] multiply(int[] array, double multiplier) {
        for (int i = 0; i < array.length; i++) {
            array[i] = (int) Math.round(array[i] * multiplier);
        }
        return array;
    }

    /**
     * Multiplies every element in a float array by a constant double.
     *
     * @param array      The input array.
     * @param multiplier The constant to multiply by.
     * @return The input array times the multiplier.
     */
    public static float[] multiply(float[] array, double multiplier) {
        for (int i = 0; i < array.length; i++) {
            array[i] = array[i] * (float) multiplier;
        }
        return array;
    }

    /**
     * Multiplies every element in a long array by a constant double.
     *
     * @param array      The input array.
     * @param multiplier The constant to multiply by.
     * @return The input array times the multiplier.
     */
    public static long[] multiply(long[] array, double multiplier) {
        for (int i = 0; i < array.length; i++) {
            array[i] = Math.round(array[i] * multiplier);
        }
        return array;
    }

    /**
     * Divides every element in a double array by a constant double.
     *
     * @param array   The input array.
     * @param divisor The constant to divide by.
     * @return The input array divided by the divisor.
     * @throws ArithmeticException Throws this exception if you try and divide by zero.
     */
    public static double[] divide(double[] array, double divisor) {
        ExceptionChecker.assertNotEqual(divisor, 0.0, new ArithmeticException("You can't divide by zero."));
        return multiply(array, 1.0 / divisor);
    }

    /**
     * Divides every element in an integer array by a constant integer.
     *
     * @param array   The input array.
     * @param divisor The constant to divide by.
     * @return The input array divided by the divisor.
     * @throws ArithmeticException Throws this exception if you try and divide by zero.
     */
    public static int[] divide(int[] array, int divisor) {
        ExceptionChecker.assertNotEqual(divisor, 0, new ArithmeticException("You can't divide by zero."));
        return multiply(array, 1.0 / divisor);
    }

    /**
     * Divides every element in a float array by a constant float.
     *
     * @param array   The input array.
     * @param divisor The constant to divide by.
     * @return The input array divided by the divisor.
     * @throws ArithmeticException Throws this exception if you try and divide by zero.
     */
    public static float[] divide(float[] array, float divisor) {
        ExceptionChecker.assertNotEqual(divisor, 0.0f, new ArithmeticException("You can't divide by zero."));
        return multiply(array, 1.0 / divisor);
    }

    /**
     * Divides every element in a long array by a constant long.
     *
     * @param array   The input array.
     * @param divisor The constant to divide by.
     * @return The input array divided by the divisor.
     * @throws ArithmeticException Throws this exception if you try and divide by zero.
     */
    public static long[] divide(long[] array, long divisor) {
        ExceptionChecker.assertNotEqual(divisor, 0L, new ArithmeticException("You can't divide by zero."));
        return multiply(array, 1.0/divisor);
    }

    /**
     * Divides every element in a integer array by a constant double and rounds it to an integer.
     *
     * @param array   The input array.
     * @param divisor The constant to divide by.
     * @return The input array divided by the divisor.
     * @throws ArithmeticException Throws this exception if you try and divide by zero.
     */
    public static int[] divide(int[] array, double divisor) {
        ExceptionChecker.assertNotEqual(divisor, 0.0, new ArithmeticException("You can't divide by zero."));
        return multiply(array, 1.0/divisor);
    }

    /**
     * Divides every element in a float array by a constant double.
     *
     * @param array   The input array.
     * @param divisor The constant to divide by.
     * @return The input array divided by the divisor.
     * @throws ArithmeticException Throws this exception if you try and divide by zero.
     */
    public static float[] divide(float[] array, double divisor) {
        ExceptionChecker.assertNotEqual(divisor, 0.0, new ArithmeticException("You can't divide by zero."));
        return multiply(array, 1.0 / divisor);
    }

    /**
     * Divides every element in a long array by a constant double.
     *
     * @param array   The input array.
     * @param divisor The constant to divide by.
     * @return The input array divided by the divisor.
     * @throws ArithmeticException Throws this exception if you try and divide by zero.
     */
    public static long[] divide(long[] array, double divisor) {
        ExceptionChecker.assertNotEqual(divisor, 0.0, new ArithmeticException("You can't divide by zero."));
        return multiply(array, 1.0 / divisor);
    }

    /**
     * Takes the absolute value of every element in a double array.
     *
     * @param array The input array.
     * @return The absolute value of the input array.
     */
    public static double[] abs(double[] array) {
        double[] output = new double[array.length];
        for (int i = 0; i < array.length; i++) {
            output[i] = Math.abs(array[i]);
        }
        return output;
    }

    /**
     * Takes the absolute value of every element in an integer array.
     *
     * @param array The input array.
     * @return The absolute value of the input array.
     */
    public static int[] abs(int[] array) {
        int[] output = new int[array.length];
        for (int i = 0; i < array.length; i++) {
            output[i] = Math.abs(array[i]);
        }
        return output;
    }

    /**
     * Takes the absolute value of every element in a float array.
     *
     * @param array The input array.
     * @return The absolute value of the input array.
     */
    public static float[] abs(float[] array) {
        float[] output = new float[array.length];
        for (int i = 0; i < array.length; i++) {
            output[i] = Math.abs(array[i]);
        }
        return output;
    }

    /**
     * Takes the absolute value of every element in a long array.
     *
     * @param array The input array.
     * @return The absolute value of the input array.
     */
    public static long[] abs(long[] array) {
        long[] output = new long[array.length];
        for (int i = 0; i < array.length; i++) {
            output[i] = Math.abs(array[i]);
        }
        return output;
    }

    /**
     * Checks an array for duplicate values.
     *
     * @param array The array to check.
     * @param <T>   The datatype of the array.
     * @return Whether the array contains any duplicate values.
     */
    public static <T> boolean checkForDuplicates(T[] array) {
        Set<T> set = new HashSet<>();
        for (T element : array) {
            if (set.contains(element)) return true;
            set.add(element);
        }
        return false;
    }

    /**
     * Removes all duplicate values from the given array.
     *
     * @param array The array to remove duplicates from.
     * @param <T>   The datatype of the array.
     * @return The input array with no duplicate values.
     */
    public static <T> T[] removeDuplicates(T[] array) {
        Set<T> set = new LinkedHashSet<>(Arrays.asList(array));

        List<T> lst = new ArrayList<>(set);

        T[] arrOut = slice(array.clone(), 0, lst.size() - 1);
        for (int i = 0; i < lst.size(); i++) {
            arrOut[i] = lst.get(i);
        }
        return arrOut;
    }

    /**
     * Adds two double arrays.
     *
     * @param list1 The first array.
     * @param list2 The second array.
     * @return The sum of the two arrays.
     * @throws HALMathException Throws this exception if the arrays are different sizes.
     */
    public static double[] add(double[] list1, double[] list2) {
        ExceptionChecker.assertTrue(list1.length == list2.length, new HALMathException("Arrays are different sizes, can't be subtracted"));

        double[] output = new double[list1.length];
        for (int i = 0; i < list1.length; i++) {
            output[i] = list1[i] + list2[i];
        }
        return output;
    }


    /**
     * Adds two integer arrays.
     *
     * @param list1 The first array.
     * @param list2 The second array.
     * @return The sum of the two arrays.
     * @throws HALMathException Throws this exception if the arrays are different sizes.
     */
    public static int[] add(int[] list1, int[] list2) {
        ExceptionChecker.assertTrue(list1.length == list2.length, new HALMathException("Arrays are different sizes, can't be subtracted"));

        int[] output = new int[list1.length];
        for (int i = 0; i < list1.length; i++) {
            output[i] = list1[i] + list2[i];
        }
        return output;
    }

    /**
     * Subtracts two double arrays.
     *
     * @param list1 The first array.
     * @param list2 The array to subtract from the first array.
     * @return The first array minus the second array.
     */
    public static double[] subtract(double[] list1, double[] list2) {
        return add(multiply(list2.clone(), -1), list1);
    }

    /**
     * Subtracts two integer arrays.
     *
     * @param list1 The first array.
     * @param list2 The array to subtract from the first array.
     * @return The first array minus the second array.
     */
    public static int[] subtract(int[] list1, int[] list2) {
        return add(multiply(list2.clone(), -1), list1);
    }

    /**
     * Calculates the absolute difference between two double arrays.
     *
     * @param list1 The first array.
     * @param list2 The array to subtract from the first array.
     * @return The absolute difference between the arrays.
     */
    public static double[] absdiff(double[] list1, double[] list2) {
        return abs(subtract(list1, list2));
    }

    /**
     * Calculates the absolute difference between two integer arrays.
     *
     * @param list1 The first array.
     * @param list2 The array to subtract from the first array.
     * @return The absolute difference between the arrays.
     */
    public static int[] absdiff(int[] list1, int[] list2) {
        return abs(subtract(list1, list2));
    }

    /**
     * Uses HALMathUtil to fix floating point errors in the given double array.
     *
     * @param list The array to fix.
     */
    public static void floatingPointFix(double[] list) {
        for (int i = 0; i < list.length; i++) {
            list[i] = HALMathUtil.floatingPointFix(list[i]);
        }
    }
}