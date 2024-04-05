// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.murraybridgebunyips.bunyipslib.external.units;

import org.murraybridgebunyips.bunyipslib.external.units.collections.LongToObjectHashMap;

import java.util.Objects;

/**
 * Generic combinatory unit type that represents the proportion of one unit to another, such as
 * Meters per Second or Radians per Celsius.
 *
 * <p>Note: {@link Velocity} is used to represent the velocity dimension, rather than {@code
 * Per<Distance, Time>}.
 *
 * @param <N> the type of the numerator unit
 * @param <D> the type of the denominator unit
 */
public class Per<N extends Unit<N>, D extends Unit<D>> extends Unit<Per<N, D>> {
    /**
     * Keep a cache of created instances so expressions like Volts.per(Meter) don't do any allocations
     * after the first.
     */
    @SuppressWarnings("rawtypes")
    private static final LongToObjectHashMap<Per> cache = new LongToObjectHashMap<>();
    private final N numerator;
    private final D denominator;

    /**
     * Creates a new proportional unit derived from the ratio of one unit to another. Consider using
     * {@link #combine} instead of manually calling this constructor.
     *
     * @param baseType    the base type representing the unit ratio
     * @param numerator   the numerator unit
     * @param denominator the denominator unit
     */
    protected Per(Class<? extends Per<N, D>> baseType, N numerator, D denominator) {
        super(
                baseType,
                numerator.toBaseUnits(1) / denominator.toBaseUnits(1),
                numerator.name() + " per " + denominator.name(),
                numerator.symbol() + "/" + denominator.symbol());
        this.numerator = numerator;
        this.denominator = denominator;
    }

    /**
     * Creates a new Per unit derived from an arbitrary numerator and time denominator units. Using a
     * denominator with a unit of time is discouraged; use {@link Velocity} instead.
     *
     * <pre>
     *   Per.combine(Volts, Meters) // possible PID constant
     * </pre>
     *
     * <p>It's recommended to use the convenience function {@link Unit#per(Unit)} instead of calling
     * this factory directly.
     *
     * @param <N>         the type of the numerator unit
     * @param <D>         the type of the denominator unit
     * @param numerator   the numerator unit
     * @param denominator the denominator for unit time
     * @return the combined unit
     */
    @SuppressWarnings({"unchecked", "rawtypes"})
    public static <N extends Unit<N>, D extends Unit<D>> Per<N, D> combine(
            N numerator, D denominator) {
        long key =
                ((long) numerator.hashCode()) << 32L | (denominator.hashCode() & 0xFFFFFFFFL);

        Per existing = cache.get(key);
        if (existing != null) {
            return existing;
        }

        Per newUnit = new Per<N, D>((Class) Per.class, numerator, denominator);
        cache.put(key, newUnit);
        return newUnit;
    }

    /**
     * Gets the numerator unit. For a {@code Per<A, B>}, this will return the {@code A} unit.
     *
     * @return the numerator unit
     */
    public N numerator() {
        return numerator;
    }

    /**
     * Gets the denominator unit. For a {@code Per<A, B>}, this will return the {@code B} unit.
     *
     * @return the denominator unit
     */
    public D denominator() {
        return denominator;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }
        if (!super.equals(o)) {
            return false;
        }
        Per<?, ?> per = (Per<?, ?>) o;
        return Objects.equals(numerator, per.numerator)
                && Objects.equals(denominator, per.denominator);
    }

    @Override
    public int hashCode() {
        return Objects.hash(super.hashCode(), numerator, denominator);
    }
}
