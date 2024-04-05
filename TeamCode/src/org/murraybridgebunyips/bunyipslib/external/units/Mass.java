// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.murraybridgebunyips.bunyipslib.external.units;

/**
 * Unit of mass dimension.
 *
 * <p>This is the base type for units of mass dimension. It is also used to specify the dimension
 * for {@link Measure}: {@code Measure<Mass>}.
 *
 * <p>Actual units (such as {@link Units#Grams} and {@link Units#Pounds}) can be found in the {@link
 * Units} class.
 */
@SuppressWarnings("SameParameterValue")
public class Mass extends Unit<Mass> {
    /**
     * Creates a new unit with the given name and multiplier to the base unit.
     */
    Mass(double baseUnitEquivalent, String name, String symbol) {
        super(Mass.class, baseUnitEquivalent, name, symbol);
    }

    Mass(UnaryFunction toBaseConverter, UnaryFunction fromBaseConverter, String name, String symbol) {
        super(Mass.class, toBaseConverter, fromBaseConverter, name, symbol);
    }
}
