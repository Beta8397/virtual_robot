package org.murraybridgebunyips.imposter.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.CommandBasedBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.drive.TriDeadwheelMecanumDrive;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.subsystems.Switch;
import org.murraybridgebunyips.bunyipslib.tasks.*;
import org.murraybridgebunyips.bunyipslib.tasks.groups.SequentialTaskGroup;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

/** bunyipslib virtual testing ground */
@TeleOp(name = "TeleOp w/ Cmd", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterTeleOpCmd extends CommandBasedBunyipsOpMode {
    private final ImposterConfig config = new ImposterConfig();
    private Switch backServo;
    private MecanumDrive drive;

    @Override
    protected void onInitialise() {
        config.init();
        drive = new TriDeadwheelMecanumDrive(config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor, config.localizerCoefficients, config.enc_left, config.enc_right, config.enc_x);
        backServo = new Switch(config.back_servo);
        addSubsystems(drive, backServo);
        drive.disable();
    }

    @Override
    protected void assignCommands() {
        scheduler().when(backServo::isOpen).runDebounced(drive::enable);
        driver().whenPressed(Controls.A).run(backServo.openTask());
        drive.setDefaultTask(new HolonomicDriveTask(gamepad1, drive, () -> false));
    }
}