// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.murraybridgebunyips.bunyipslib.external.units;

/**
 * Unit of power dimension.
 *
 * <p>This is the base type for units of power dimension. It is also used to specify the dimension
 * for {@link Measure}: {@code Measure<Power>}.
 *
 * <p>Actual units (such as {@link Units#Watts} and {@link Units#Horsepower}) can be found in the
 * {@link Units} class.
 */
public class Power extends Unit<Power> {
    Power(Power baseUnit, double baseUnitEquivalent, String name, String symbol) {
        super(baseUnit, baseUnitEquivalent, name, symbol);
    }

    Power(
            Power baseUnit,
            UnaryFunction toBaseConverter,
            UnaryFunction fromBaseConverter,
            String name,
            String symbol) {
        super(baseUnit, toBaseConverter, fromBaseConverter, name, symbol);
    }
}
