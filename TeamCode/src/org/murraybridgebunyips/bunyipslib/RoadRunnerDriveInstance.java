package org.murraybridgebunyips.bunyipslib;

import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

/**
 * Used to override for a drive instance for the {@link RoadRunner} interface.
 *
 * @author Lucas Bubner, 2024
 * @see RoadRunner
 */
@FunctionalInterface
interface RoadRunnerDriveInstance {
    /**
     * Get the drive instance reference to be used for RoadRunner trajectories.
     * <b>Do NOT instantiate a new drive instance here, use the reference from your subsystems in your OpMode.</b>
     * (e.g. {@code return drive;}) instead of ({@code return new MecanumDrive(...);})
     *
     * @return Drive instance reference
     */
    @NotNull
    RoadRunnerDrive getDrive();
}
