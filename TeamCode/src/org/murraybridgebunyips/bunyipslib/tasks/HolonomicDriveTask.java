package org.murraybridgebunyips.bunyipslib.tasks;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.drive.CartesianMecanumDrive;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.RunForeverTask;

/**
 * Standard gamepad drive for all holonomic drivetrains.
 * Left stick controls translation, right stick controls rotation.
 * This task is designed to be used as a default task, other tasks will override it.
 * @author Lucas Bubner, 2024
 */
public class HolonomicDriveTask<T extends BunyipsSubsystem> extends RunForeverTask {
    private final T drive;
    private final Gamepad gamepad;

    public HolonomicDriveTask(Gamepad gamepad, @NotNull T mecanumDrive) {
        super(mecanumDrive, false);
        if (!(mecanumDrive instanceof MecanumDrive) && !(mecanumDrive instanceof CartesianMecanumDrive))
            throw new IllegalArgumentException("HolonomicDriveTask must be used with a holonomic drivetrain");
        this.drive = mecanumDrive;
        this.gamepad = gamepad;
    }

    @Override
    public void init() {
        // noop
    }

    @Override
    public void periodic() {
        if (drive instanceof MecanumDrive) {
            ((MecanumDrive) drive).setSpeedUsingController(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x);
        } else if (drive instanceof CartesianMecanumDrive) {
            ((CartesianMecanumDrive) drive).setSpeedUsingController(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x);
        }
    }

    @Override
    public void onFinish() {
        if (drive instanceof MecanumDrive) {
            ((MecanumDrive) drive).stop();
        } else if (drive instanceof CartesianMecanumDrive) {
            ((CartesianMecanumDrive) drive).stop();
        }
    }
}
