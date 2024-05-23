// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.murraybridgebunyips.bunyipslib.external.units;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Objects;

/**
 * Builder used for easily deriving new units from existing ones. When deriving a new unit, the base
 * unit class <strong>must</strong> redeclare the constructor {@link Unit#Unit(Unit, UnaryFunction,
 * UnaryFunction, String, String) (U, UnaryFunction, UnaryFunction, String, String)}. The unit
 * builder class will invoke this constructor automatically and build the new unit. Alternatively,
 * new units can be derived by passing an explicit constructor function to {@link
 * #make(UnitConstructorFunction)}.
 *
 * @param <U> the type of the unit
 */
public final class UnitBuilder<U extends Unit<U>> {
    private final U base;
    private UnaryFunction fromBaseVal = UnaryFunction.IDENTITY;
    private UnaryFunction toBaseVal = UnaryFunction.IDENTITY;
    private String nameVal;
    private String symbolVal;

    /**
     * Creates a new unit builder object, building off of a base unit. The base unit does not have to
     * be <i>the</i> base unit of its unit system; furlongs work just as well here as meters.
     *
     * @param base the unit to base the new unit off of
     */
    public UnitBuilder(U base) {
        this.base = Objects.requireNonNull(base, "Base unit cannot be null");
    }

    /**
     * Maps a value {@code value} in the range {@code [inMin..inMax]} to an output in the range {@code
     * [outMin..outMax]}. Inputs outside the bounds will be mapped correspondingly to outputs outside
     * the output bounds. Inputs equal to {@code inMin} will be mapped to {@code outMin}, and inputs
     * equal to {@code inMax} will similarly be mapped to {@code outMax}.
     *
     * @param value  the value to map
     * @param inMin  the minimum input value (does not have to be absolute)
     * @param inMax  the maximum input value (does not have to be absolute)
     * @param outMin the minimum output value (does not have to be absolute)
     * @param outMax the maximum output value (does not have to be absolute)
     * @return the mapped output
     */
    // NOTE: This method lives here instead of in MappingBuilder because inner classes can't
    // define static methods prior to Java 16.
    private static double mapValue(
            double value, double inMin, double inMax, double outMin, double outMax) {
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    @SuppressWarnings({"unchecked", "rawtypes"})
    private static <U extends Unit<U>> Constructor<? extends Unit<U>> getConstructor(U baseUnit)
            throws NoSuchMethodException {
        Class<? extends Unit> baseClass = baseUnit.getClass();

        Constructor<? extends Unit<U>> ctor =
                (Constructor<? extends Unit<U>>) baseClass.getDeclaredConstructor(
                        baseClass, // baseUnit
                        UnaryFunction.class, // toBaseUnits
                        UnaryFunction.class, // fromBaseUnits
                        String.class, // name
                        String.class); // symbol

        // Need to flag the constructor as accessible so we can use private, package-private,
        // and protected constructors
        ctor.setAccessible(true);

        return ctor;
    }

    /**
     * Sets the unit conversions based on a simple offset. The new unit will have its values equal to
     * (base value - offset).
     *
     * @param offset the offset
     * @return this builder
     */
    public UnitBuilder<U> offset(double offset) {
        toBaseVal = derivedValue -> derivedValue + offset;
        fromBaseVal = baseValue -> baseValue - offset;
        return this;
    }

    /**
     * Defines a mapping for values within the given input range. This method call should be
     * immediately followed by {@code .toOutputRange}, eg {@code mappingInputRange(1,
     * 2).toOutputRange(3, 4)}, which will return the unit builder for continued chaining.
     *
     * @param minBase the minimum input value (does not have to be absolute)
     * @param maxBase the maximum output value (does not have to be absolute)
     * @return a builder object used to define the output range
     */
    public MappingBuilder mappingInputRange(double minBase, double maxBase) {
        return new MappingBuilder(minBase, maxBase);
    }

    /**
     * Sets the conversion function to transform values in the base unit to values in the derived
     * unit.
     *
     * @param fromBase the conversion function
     * @return the unit builder, for continued chaining
     */
    public UnitBuilder<U> fromBase(UnaryFunction fromBase) {
        fromBaseVal = Objects.requireNonNull(fromBase, "fromBase function cannot be null");
        return this;
    }

    /**
     * Sets the conversion function to transform values in the derived unit to values in the base
     * unit.
     *
     * @param toBase the conversion function
     * @return the unit builder, for continued chaining
     */
    public UnitBuilder<U> toBase(UnaryFunction toBase) {
        toBaseVal = Objects.requireNonNull(toBase, "toBase function cannot be null");
        return this;
    }

    /**
     * Sets the name of the new unit.
     *
     * @param name the new name
     * @return the unit builder, for continued chaining
     */
    public UnitBuilder<U> named(String name) {
        nameVal = name;
        return this;
    }

    /**
     * Sets the symbol of the new unit.
     *
     * @param symbol the new symbol
     * @return the unit builder, for continued chaining
     */
    public UnitBuilder<U> symbol(String symbol) {
        symbolVal = symbol;
        return this;
    }

    /**
     * Helper for defining units that are a scalar fraction of the base unit, such as centimeters
     * being 1/100th of the base unit (meters). The fraction value is specified as the denominator of
     * the fraction, so a centimeter definition would use {@code splitInto(100)} instead of {@code
     * splitInto(1/100.0)}.
     *
     * @param fraction the denominator portion of the fraction of the base unit that a value of 1 in
     *                 the derived unit corresponds to
     * @return the unit builder, for continued chaining
     */
    public UnitBuilder<U> splitInto(double fraction) {
        if (fraction == 0) {
            throw new IllegalArgumentException("Fraction must be nonzero");
        }

        return toBase(x -> x / fraction).fromBase(b -> b * fraction);
    }

    /**
     * Helper for defining units that are a scalar multiple of the base unit, such as kilometers being
     * 1000x of the base unit (meters).
     *
     * @param aggregation the magnitude required for a measure in the base unit to equal a magnitude
     *                    of 1 in the derived unit
     * @return the unit builder, for continued chaining
     */
    public UnitBuilder<U> aggregate(double aggregation) {
        if (aggregation == 0) {
            throw new IllegalArgumentException("Aggregation amount must be nonzero");
        }

        return toBase(x -> x * aggregation).fromBase(b -> b / aggregation);
    }

    /**
     * Creates the new unit based off of the builder methods called prior, passing them to a provided
     * constructor function.
     *
     * @param constructor the function to use to create the new derived unit
     * @return the new derived unit
     * @throws NullPointerException if the unit conversions, unit name, or unit symbol were not set
     */
    public U make(UnitConstructorFunction<U> constructor) {
        Objects.requireNonNull(fromBaseVal, "fromBase function was not set");
        Objects.requireNonNull(toBaseVal, "toBase function was not set");
        Objects.requireNonNull(nameVal, "new unit name was not set");
        Objects.requireNonNull(symbolVal, "new unit symbol was not set");

        return constructor.create(
                base.getBaseUnit(),
                toBaseVal.pipeTo(base.getConverterToBase()),
                base.getConverterFromBase().pipeTo(fromBaseVal),
                nameVal,
                symbolVal);
    }

    /**
     * Creates the new unit based off of the builder methods called prior.
     *
     * @return the new derived unit
     * @throws NullPointerException if the unit conversions, unit name, or unit symbol were not set
     * @throws RuntimeException     if the base unit does not define a constructor accepting the
     *                              conversion functions, unit name, and unit symbol - in that order
     */
    @SuppressWarnings("unchecked")
    public U make() {
        return make(
                (baseUnit, toBaseUnits, fromBaseUnits, name, symbol) -> {
                    Class<?> baseClass = baseUnit.getClass();

                    try {
                        Constructor<? extends Unit<U>> ctor = getConstructor(baseUnit);
                        return (U) ctor.newInstance(baseUnit, toBaseUnits, fromBaseUnits, name, symbol);
                    } catch (InstantiationException e) {
                        throw new RuntimeException("Could not instantiate class " + baseClass.getName(), e);
                    } catch (IllegalAccessException e) {
                        throw new RuntimeException("Could not access constructor", e);
                    } catch (InvocationTargetException e) {
                        throw new RuntimeException(
                                "Constructing " + baseClass.getName() + " raised an exception", e);
                    } catch (NoSuchMethodException e) {
                        throw new RuntimeException(
                                "No compatible constructor "
                                        + baseClass.getSimpleName()
                                        + "("
                                        + baseClass.getSimpleName()
                                        + ", UnaryFunction, UnaryFunction, String, String)",
                                e);
                    }
                });
    }

    /**
     * A functional interface for constructing new units without relying on reflection.
     *
     * @param <U> the type of the unit
     */
    @FunctionalInterface
    public interface UnitConstructorFunction<U extends Unit<U>> {
        /**
         * Creates a new unit instance based on its relation to the base unit of measure.
         *
         * @param baseUnit      the base unit of the unit system
         * @param toBaseUnits   a function that converts values of the new unit to equivalent values in
         *                      terms of the base unit
         * @param fromBaseUnits a function that converts values in the base unit to equivalent values in
         *                      terms of the new unit
         * @param name          the name of the new unit
         * @param symbol        the shorthand symbol of the new unit
         * @return a new unit
         */
        U create(
                U baseUnit,
                UnaryFunction toBaseUnits,
                UnaryFunction fromBaseUnits,
                String name,
                String symbol);
    }

    /**
     * Helper class used for safely chaining mapping builder calls.
     */
    public class MappingBuilder {
        private final double minInput;
        private final double maxInput;

        private MappingBuilder(double minInput, double maxInput) {
            this.minInput = minInput;
            this.maxInput = maxInput;
        }

        /**
         * Finalizes the mapping by defining the output range.
         *
         * @param minOutput the minimum output value (does not have to be absolute)
         * @param maxOutput the maximum output value (does not have to be absolute)
         * @return the unit builder, for continued chaining
         */
        public UnitBuilder<U> toOutputRange(double minOutput, double maxOutput) {
            fromBaseVal = x -> mapValue(x, minInput, maxInput, minOutput, maxOutput);
            toBaseVal = y -> mapValue(y, minOutput, maxOutput, minInput, maxInput);
            return UnitBuilder.this;
        }
    }
}
