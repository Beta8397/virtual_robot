// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.murraybridgebunyips.bunyipslib.external.units;

import androidx.annotation.NonNull;

import java.util.Objects;

/**
 * A measure holds the magnitude and unit of some dimension, such as distance, time, or speed. An
 * immutable measure is <i>immutable</i> and <i>type safe</i>, making it easy to use in concurrent
 * situations and gives compile-time safety. Two measures with the same <i>unit</i> and
 * <i>magnitude</i> are effectively equivalent objects.
 *
 * @param <U> the unit type of the measure
 */
public class ImmutableMeasure<U extends Unit<U>> implements Measure<U> {
    private final double magnitude;
    private final double baseUnitMagnitude;
    private final U unit;

    /**
     * Creates a new immutable measure instance. This shouldn't be used directly; prefer one of the
     * factory methods instead.
     *
     * @param magnitude the magnitude of this measure
     * @param unit      the unit of this measure.
     */
    @SuppressWarnings("unchecked")
    ImmutableMeasure(double magnitude, double baseUnitMagnitude, Unit<U> unit) {
        Objects.requireNonNull(unit, "Unit cannot be null");
        this.magnitude = magnitude;
        this.baseUnitMagnitude = baseUnitMagnitude;
        this.unit = (U) unit;
    }

    /**
     * Creates a new measure in the given unit with a magnitude equal to the given one in base units.
     *
     * @param <U>               the type of the units of measure
     * @param baseUnitMagnitude the magnitude of the measure, in terms of the base unit of measure
     * @param unit              the unit of measure
     * @return a new measure
     */
    public static <U extends Unit<U>> ImmutableMeasure<U> ofBaseUnits(
            double baseUnitMagnitude, Unit<U> unit) {
        return new ImmutableMeasure<>(unit.fromBaseUnits(baseUnitMagnitude), baseUnitMagnitude, unit);
    }

    /**
     * Creates a new measure in the given unit with a magnitude in terms of that unit.
     *
     * @param <U>               the type of the units of measure
     * @param relativeMagnitude the magnitude of the measure
     * @param unit              the unit of measure
     * @return a new measure
     */
    public static <U extends Unit<U>> ImmutableMeasure<U> ofRelativeUnits(
            double relativeMagnitude, Unit<U> unit) {
        return new ImmutableMeasure<>(relativeMagnitude, unit.toBaseUnits(relativeMagnitude), unit);
    }

    /**
     * Gets the unitless magnitude of this measure.
     */
    @Override
    public double magnitude() {
        return magnitude;
    }

    @Override
    public double baseUnitMagnitude() {
        return baseUnitMagnitude;
    }

    /**
     * Gets the units of this measure.
     */
    @Override
    public U unit() {
        return unit;
    }

    /**
     * Checks for <i>object equality</i>. To check if two measures are <i>equivalent</i>, use {@link
     * #isEquivalent(Measure) isEquivalent}.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof Measure)) {
            return false;
        }
        Measure<?> that = (Measure<?>) o;
        return Objects.equals(unit, that.unit()) && baseUnitMagnitude == that.baseUnitMagnitude();
    }

    @Override
    public int hashCode() {
        return Objects.hash(magnitude, unit);
    }

    @Override
    public Measure<U> copy() {
        return this; // Already immutable, no need to allocate a new object
    }

    @NonNull
    @Override
    public String toString() {
        return toShortString();
    }
}
