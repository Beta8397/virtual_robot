package org.murraybridgebunyips.imposter.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.CommandBasedBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.RoadRunner;
import org.murraybridgebunyips.bunyipslib.drive.TriDeadwheelMecanumDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.subsystems.DualServos;
import org.murraybridgebunyips.bunyipslib.tasks.DynamicTask;
import org.murraybridgebunyips.bunyipslib.tasks.HolonomicDriveTask;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

@TeleOp
public class ImposterAugmented extends CommandBasedBunyipsOpMode implements RoadRunner {
    private ImposterConfig config = new ImposterConfig();
    private TriDeadwheelMecanumDrive drive;

    @Override
    protected void onInitialise() {
        config.init();
        drive = new TriDeadwheelMecanumDrive(config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor, config.localizerCoefficients, config.enc_left, config.enc_right, config.enc_x);
    }

    @Override
    protected void assignCommands() {
        drive.setDefaultTask(new HolonomicDriveTask(gamepad1, drive, () -> false));
        driver()
                .whenPressed(Controls.A)
                .run(new DynamicTask(() ->
                        makeTrajectory(drive.getPoseEstimate())
                                .lineTo(new Vector2d())
                                .buildTask(false))
                );
    }

    @NotNull
    @Override
    public RoadRunnerDrive getDrive() {
        return drive;
    }
}
