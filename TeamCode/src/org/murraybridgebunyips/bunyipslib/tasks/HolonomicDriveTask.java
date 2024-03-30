package org.murraybridgebunyips.bunyipslib.tasks;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controller;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.drive.CartesianMecanumDrive;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Standard gamepad drive for all holonomic drivetrains.
 * Left stick controls translation, right stick controls rotation.
 * This task is designed to be used as a default task, other tasks will override it.
 *
 * @param <T> The type of the MecanumDrive to use
 * @author Lucas Bubner, 2024
 */
public class HolonomicDriveTask<T extends BunyipsSubsystem> extends ForeverTask {
    private final T drive;
    private final Supplier<Float> x;
    private final Supplier<Float> y;
    private final Supplier<Float> r;
    private final BooleanSupplier fieldCentricEnabled;
    private final DoubleSupplier multiplierSupplier;

    /**
     * Constructor for HolonomicDriveTask.
     *
     * @param controller          The gamepad to use for driving
     * @param mecanumDrive        The MecanumDrive to use for driving
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled,
     *                            this will only work on a MecanumDrive that supports dynamic field-centric
     *                            drive switching, such as the RoadRunner-integrated MecanumDrive
     */
    public HolonomicDriveTask(Controller controller, @NonNull T mecanumDrive, BooleanSupplier fieldCentricEnabled) {
        this(controller, mecanumDrive, () -> 1.0, fieldCentricEnabled);
    }

    /**
     * Constructor for HolonomicDriveTask.
     *
     * @param xSupplier           The supplier for the x-axis input
     * @param ySupplier           The supplier for the y-axis input
     * @param rSupplier           The supplier for the rotation input
     * @param mecanumDrive        The MecanumDrive to use for driving
     * @param multiplierSupplier  Supply a multiplier that will be applied to the inputs of each joystick.
     *                            Useful when in use with the InputMultiplier.
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled,
     *                            this will only work on a MecanumDrive that supports dynamic field-centric
     *                            drive switching, such as the RoadRunner-integrated MecanumDrive
     */
    public HolonomicDriveTask(Supplier<Float> xSupplier, Supplier<Float> ySupplier, Supplier<Float> rSupplier, @NotNull T mecanumDrive, DoubleSupplier multiplierSupplier, BooleanSupplier fieldCentricEnabled) {
        super(mecanumDrive, false);
        if (!(mecanumDrive instanceof MecanumDrive) && !(mecanumDrive instanceof CartesianMecanumDrive))
            throw new EmergencyStop("HolonomicDriveTask must be used with a holonomic drivetrain");
        drive = mecanumDrive;
        x = xSupplier;
        y = ySupplier;
        r = rSupplier;
        this.multiplierSupplier = multiplierSupplier;
        this.fieldCentricEnabled = fieldCentricEnabled;
    }


    /**
     * Constructor for HolonomicDriveTask using a default Mecanum binding.
     *
     * @param driver              The gamepad to use for driving
     * @param mecanumDrive        The MecanumDrive to use for driving
     * @param multiplierSupplier  Supply a multiplier that will be applied to the inputs of each joystick.
     *                            Useful when in use with the InputMultiplier.
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled,
     *                            this will only work on a MecanumDrive that supports dynamic field-centric
     *                            drive switching, such as the RoadRunner-integrated MecanumDrive
     */
    public HolonomicDriveTask(Gamepad driver, @NotNull T mecanumDrive, DoubleSupplier multiplierSupplier, BooleanSupplier fieldCentricEnabled) {
        this(() -> driver.left_stick_x, () -> driver.left_stick_y, () -> driver.right_stick_x, mecanumDrive, multiplierSupplier, fieldCentricEnabled);
    }

    @Override
    protected void init() {
        // no-op
    }

    @Override
    protected void periodic() {
        if (drive instanceof MecanumDrive) {
            if (fieldCentricEnabled.getAsBoolean()) {
                ((MecanumDrive) drive).setSpeedUsingControllerFieldCentric(
                        x.get() * multiplierSupplier.getAsDouble(),
                        y.get() * multiplierSupplier.getAsDouble(),
                        r.get() * multiplierSupplier.getAsDouble()
                );
            } else {
                ((MecanumDrive) drive).setSpeedUsingController(
                        x.get() * multiplierSupplier.getAsDouble(),
                        y.get() * multiplierSupplier.getAsDouble(),
                        r.get() * multiplierSupplier.getAsDouble()
                );
            }
        } else if (drive instanceof CartesianMecanumDrive) {
            ((CartesianMecanumDrive) drive).setSpeedUsingController(
                    x.get() * multiplierSupplier.getAsDouble(),
                    y.get() * multiplierSupplier.getAsDouble(),
                    r.get() * multiplierSupplier.getAsDouble()
            );
        }
    }

    @Override
    protected void onFinish() {
        if (drive instanceof MecanumDrive) {
            ((MecanumDrive) drive).stop();
        } else if (drive instanceof CartesianMecanumDrive) {
            ((CartesianMecanumDrive) drive).stop();
        }
    }
}
