package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.function.Consumer;

/**
 * Represents an object reference that may be reassigned.
 *
 * @param <V> the type of the reference
 * @author Lucas Bubner, 2024
 */
public class Reference<V> {
    @Nullable
    private volatile V value;

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
     * Gets the value of the reference.
     * If it is null, a NullPointerException is thrown.
     *
     * @return the value of the reference, never null
     */
    @NonNull
    public V require() {
        V val = value;
        if (val == null) {
            throw new NullPointerException("Reference value is null");
        }
        return val;
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
     * Sets the value of the reference if it is not already set.
     *
     * @param value the new value of the reference
     */
    public void setIfNotPresent(@Nullable V value) {
        if (this.value == null) {
            this.value = value;
        }
    }

    /**
     * Sets the value of the reference if it is already set.
     *
     * @param value the new value of the reference
     */
    public void setIfPresent(@Nullable V value) {
        if (this.value != null) {
            this.value = value;
        }
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
     * Runs the consumer on the value of the reference if it is not null.
     *
     * @param consumer the consumer to run
     */
    public void ifPresent(Consumer<V> consumer) {
        if (value != null) {
            consumer.accept(value);
        }
    }

    /**
     * Runs the runnable on the value of the reference if it is null.
     *
     * @param runnable the runnable to run if the reference is null
     */
    public void ifNotPresent(Runnable runnable) {
        if (value == null) {
            runnable.run();
        }
    }

    /**
     * Runs the consumer on the value of the reference if it is not null.
     * If the reference is null, the given runnable is run.
     *
     * @param consumer the consumer to run
     * @param runnable the runnable to run if the reference is null
     */
    public void ifPresentOrElse(Consumer<V> consumer, Runnable runnable) {
        if (value != null) {
            consumer.accept(value);
        } else {
            runnable.run();
        }
    }

    /**
     * Returns the value of the reference if it is not null, otherwise returns the given value.
     *
     * @param other the value to return if the reference is null
     * @return the value of the reference if it is not null, otherwise the given value
     */
    public V getOrElse(V other) {
        return value == null ? other : value;
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
        V val = value;
        return val == null ? "null" : val.toString();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Reference) {
            Reference<?> other = (Reference<?>) obj;
            return value == other.value;
        }
        return false;
    }

    /**
     * Returns the hash code of the reference.
     *
     * @return the hash code of the reference, 0 if the reference is null
     */
    @Override
    public int hashCode() {
        V val = value;
        return val == null ? 0 : val.hashCode();
    }
}