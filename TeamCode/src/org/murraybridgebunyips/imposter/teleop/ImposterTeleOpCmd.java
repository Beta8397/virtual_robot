package org.murraybridgebunyips.imposter.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.murraybridgebunyips.bunyipslib.CommandBasedBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.drive.TriDeadwheelMecanumDrive;
import org.murraybridgebunyips.bunyipslib.tasks.*;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

@TeleOp(name = "TeleOp w/ Cmd", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterTeleOpCmd extends CommandBasedBunyipsOpMode {
    private ImposterConfig config = new ImposterConfig();
    private MecanumDrive drive;

    @Override
    protected void onInitialisation() {
        config.init();
        drive = new TriDeadwheelMecanumDrive(config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor, config.localizerCoefficients, config.enc_left, config.enc_right, config.enc_x);
        addSubsystems(drive);
    }

    @Override
    protected void assignCommands() {
        drive.setDefaultTask(new HolonomicDriveTask<>(gamepad1, drive, () -> false));
    }
}
