package org.murraybridgebunyips.bunyipslib.tasks;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.Controller;
import org.murraybridgebunyips.bunyipslib.drive.TankDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;

/**
 * Standard gamepad drive for all differential drivetrains.
 * Left stick Y controls forward translation, right stick controls rotation.
 * This task is designed to be used as a default task, other tasks will override it.
 *
 * @author Lucas Bubner, 2024
 */
public class DifferentialDriveTask extends ForeverTask {
    private final TankDrive drive;
    private final Gamepad gamepad;

    /**
     * Constructs a new DifferentialDriveTask.
     *
     * @param gamepad the gamepad to use
     * @param drive   the drive to control
     */
    public DifferentialDriveTask(Gamepad gamepad, @NotNull TankDrive drive) {
        super(drive, false);
        this.drive = drive;
        this.gamepad = gamepad;
    }

    @Override
    protected void init() {
        // no-op
    }

    @Override
    protected void periodic() {
        drive.setWeightedDrivePower(
                Controller.makeRobotPose(gamepad.left_stick_y, 0, gamepad.right_stick_x)
        );
    }

    @Override
    protected void onFinish() {
        drive.stop();
    }
}
