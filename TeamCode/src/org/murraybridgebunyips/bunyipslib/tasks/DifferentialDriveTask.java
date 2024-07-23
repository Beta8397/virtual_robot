package org.murraybridgebunyips.bunyipslib.tasks;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.Controls;
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
        this.drive = drive;
        this.gamepad = gamepad;
        onSubsystem(drive, false);
        withName("Differential Drive Control");
    }

    @Override
    protected void periodic() {
        drive.setWeightedDrivePower(
                Controls.makeRobotPose(gamepad.left_stick_y, 0, gamepad.right_stick_x)
        );
    }

    @Override
    protected void onFinish() {
        drive.stop();
    }
}
