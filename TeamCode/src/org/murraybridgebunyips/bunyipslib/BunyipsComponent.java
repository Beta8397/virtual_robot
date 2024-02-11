package org.murraybridgebunyips.bunyipslib;

/**
 * Base class for components used in BunyipsOpModes.
 *
 * @author Lucas Bubner, 2022
 */
public abstract class BunyipsComponent {
    protected final BunyipsOpMode opMode;

    protected BunyipsComponent(BunyipsOpMode opMode) {
        this.opMode = opMode;
    }
}
