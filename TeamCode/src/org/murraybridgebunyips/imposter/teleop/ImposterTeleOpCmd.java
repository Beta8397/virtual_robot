package org.murraybridgebunyips.imposter.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.CommandBasedBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.drive.TriDeadwheelMecanumDrive;
import org.murraybridgebunyips.bunyipslib.tasks.HolonomicDriveTask;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

@TeleOp(name = "TeleOp w/ Cmd", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterTeleOpCmd extends CommandBasedBunyipsOpMode {
    private ImposterConfig config = new ImposterConfig();
    private MecanumDrive drive;

    @Override
    protected void onInitialisation() {
        config.init();
        drive = new TriDeadwheelMecanumDrive(config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor, config.localizerCoefficients, config.enc_left, config.enc_right, config.enc_x);
    }

    @Override
    protected BunyipsSubsystem[] setSubsystems() {
        return new BunyipsSubsystem[] {
                drive
        };
    }

    @Override
    protected void assignCommands() {
        drive.setDefaultTask(new HolonomicDriveTask<>(gamepad1, drive, () -> false));
    }
}
