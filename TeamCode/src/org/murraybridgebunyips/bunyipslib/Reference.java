package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/**
 * Represents an object reference that may be reassigned.
 *
 * @param <V> the type of the reference
 * @author Lucas Bubner, 2024
 */
public class Reference<V> {
    @Nullable
    private V value;

    /**
     * Creates a new reference with a null value.
     */
    public Reference() {
        this(null);
    }

    /**
     * Creates a new reference with the given value.
     *
     * @param value the value of the reference
     */
    public Reference(@Nullable V value) {
        this.value = value;
    }

    /**
     * Creates a new reference with the given value.
     *
     * @param value the value of the reference
     * @param <V>   the type of the reference
     * @return a new reference with the given value
     */
    public static <V> Reference<V> of(@Nullable V value) {
        return new Reference<>(value);
    }

    /**
     * Creates a new reference with a null value.
     *
     * @param <V> the type of the reference
     * @return a new reference with a null value
     */
    public static <V> Reference<V> empty() {
        return new Reference<>();
    }

    /**
     * Gets the value of the reference.
     *
     * @return the value of the reference
     */
    @Nullable
    public V get() {
        return value;
    }

    /**
     * Sets the value of the reference.
     *
     * @param value the new value of the reference
     */
    public void set(@Nullable V value) {
        this.value = value;
    }

    /**
     * Checks if the reference is null.
     *
     * @return true if the reference is null, false otherwise
     */
    public boolean isNull() {
        return value == null;
    }

    /**
     * Checks if the reference is not null.
     *
     * @return true if the reference is not null, false otherwise
     */
    public boolean isNotNull() {
        return value != null;
    }

    /**
     * Clears the reference, setting it to null.
     */
    public void clear() {
        value = null;
    }

    /**
     * Returns a string representation of the reference.
     *
     * @return a string representing the reference, or "null" if the reference is null
     */
    @NonNull
    @Override
    public String toString() {
        return value == null ? "null" : value.toString();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Reference) {
            Reference<?> other = (Reference<?>) obj;
            return value == other.value;
        }
        return false;
    }
}
