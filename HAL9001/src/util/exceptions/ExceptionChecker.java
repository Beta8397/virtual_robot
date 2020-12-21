package util.exceptions;

/**
 * An class used to easily throw exceptions when a condition is met or not met.
 * <p>
 * Creation Date: 12/20/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see RuntimeException
 * @since 1.0.6
 */
public class ExceptionChecker {

    /**
     * A private constructor for ExceptionChecker so that it becomes a static class.
     */
    private ExceptionChecker() {
    }

    /**
     * Throws an exception if a given condition is not true.
     *
     * @param condition The condition that must be false to throw the exception.
     * @param exception The exception to throw if the condition is false.
     * @see RuntimeException
     */
    public static void assertTrue(boolean condition, RuntimeException exception) {
        if (!condition) throw exception;
    }

    /**
     * Throws an exception if a given condition is not false.
     *
     * @param condition The condition that must be false to throw the exception.
     * @param exception The exception to throw if the condition is true.
     * @see RuntimeException
     */
    public static void assertFalse(boolean condition, RuntimeException exception) {
        assertTrue(!condition, exception);
    }

    /**
     * Throws an exception if the given object is not null.
     *
     * @param object    The object to check.
     * @param exception The exception to throw if the object is null.
     * @see RuntimeException
     */
    public static void assertNonNull(Object object, RuntimeException exception) {
        assertFalse(object == null, exception);
    }

    /**
     * Throws an exception if the given object is not null.
     *
     * @param object    The object that should be null.
     * @param exception The exception to throw if the object is not null.
     * @see RuntimeException
     */
    public static void assertNull(Object object, RuntimeException exception) {
        assertTrue(object == null, exception);
    }

    /**
     * Throws an exception if two objects are not equal.
     *
     * @param obj1      The first object.
     * @param obj2      The second object.
     * @param exception The exception to throw if the two objects are not equal.
     * @see RuntimeException
     */
    public static void assertEqual(Object obj1, Object obj2, RuntimeException exception) {
        assertTrue(obj1.equals(obj2), exception);
    }

    /**
     * Throws an exception if two objects are equal.
     *
     * @param obj1      The first object.
     * @param obj2      The second object.
     * @param exception The exception to throw if the objects are equal.
     * @see RuntimeException
     */
    public static void assertNotEqual(Object obj1, Object obj2, RuntimeException exception) {
        assertFalse(obj1.equals(obj2), exception);
    }
}