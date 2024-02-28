package org.murraybridgebunyips.bunyipslib;

/**
 * Base class for components used with a BunyipsOpMode.
 * This allows injection of the OpMode into the component, and provides a common base for all components.
 *
 * @author Lucas Bubner, 2024
 */
public abstract class BunyipsComponent {
    // Since the OpMode is static, this means ** ALL components MUST be instantiated in the init phase ** , and not
    // in the constructor/member fields. You will experience extremely strange behaviour if you do not follow this.
    // This should not be a problem as all components are usually instantiated in the init phase anyway due to
    // the need to access motors and other hardware at runtime.
    protected final BunyipsOpMode opMode = BunyipsOpMode.getInstance();
}
