package org.murraybridgebunyips.imposter.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.CommandBasedBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.drive.TriDeadwheelMecanumDrive;
import org.murraybridgebunyips.bunyipslib.tasks.*;
import org.murraybridgebunyips.bunyipslib.tasks.groups.SequentialTaskGroup;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

@TeleOp(name = "TeleOp w/ Cmd", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterTeleOpCmd extends CommandBasedBunyipsOpMode {
    private ImposterConfig config = new ImposterConfig();
    private MecanumDrive drive;
    private Subsystem1 ss1;

    @Override
    protected void onInitialisation() {
        config.init();
        drive = new TriDeadwheelMecanumDrive(config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor, config.localizerCoefficients, config.enc_left, config.enc_right, config.enc_x);
        ss1 = new Subsystem1();
        addSubsystems(drive, ss1);
    }

    @Override
    protected void assignCommands() {
        drive.setDefaultTask(new HolonomicDriveTask<>(gamepad1, drive, () -> false));
        driver().whenPressed(Controls.A)
                .run(new ContinuousTask(() -> addTelemetry("toggle is on")))
                .finishingWhen(() -> gamepad1.getDebounced(Controls.A));
    }

    class Thing extends SequentialTaskGroup {
        public Thing(Subsystem1 a, MecanumDrive b) {
            super(
                    new RunForTask(1, () -> getInstance().addTelemetry("1 second"), a, false),
                    new RunForTask(3, () -> {}, b, false),
                    new RunForTask(1, () -> {}, a, false)
            );
        }
    }

    class Subsystem1 extends BunyipsSubsystem {

        @Override
        protected void periodic() {

        }
    }
}