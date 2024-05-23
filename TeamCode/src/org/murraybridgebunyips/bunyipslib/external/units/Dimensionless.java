// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.murraybridgebunyips.bunyipslib.external.units;

/**
 * A type of unit that corresponds to raw values and not any physical dimension, such as percentage.
 */
public class Dimensionless extends Unit<Dimensionless> {
    /**
     * @noinspection SameParameterValue
     */
    Dimensionless(Dimensionless baseUnit, double baseUnitEquivalent, String name, String symbol) {
        super(baseUnit, baseUnitEquivalent, name, symbol);
    }

    Dimensionless(
            Dimensionless baseUnit,
            UnaryFunction toBaseConverter,
            UnaryFunction fromBaseConverter,
            String name,
            String symbol) {
        super(baseUnit, toBaseConverter, fromBaseConverter, name, symbol);
    }
}
