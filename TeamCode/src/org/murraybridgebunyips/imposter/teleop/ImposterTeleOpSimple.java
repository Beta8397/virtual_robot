package org.murraybridgebunyips.imposter.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.murraybridgebunyips.bunyipslib.*;
import org.murraybridgebunyips.bunyipslib.tasks.InstantTask;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

import java.util.ArrayList;

@TeleOp(name = "TeleOp", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterTeleOpSimple extends BunyipsOpMode {
    private final ImposterConfig config = new ImposterConfig();
    private MecanumDrive drive;
    private Scheduler scheduler = new Scheduler(this);

    @Override
    protected void onInit() {
        config.init(this);
        drive = new TriDeadwheelMecanumDrive(this, config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor, config.localizerCoefficients, config.enc_left, config.enc_right, config.enc_x);
        scheduler.addSubsystems(drive);
        scheduler.whenPressed(Controller.User.ONE, Controller.A)
                .run(new InstantTask(() -> addTelemetry("Hello world")))
                .immediately();
    }

    @Override
    protected void activeLoop() {
        scheduler.run();
    }
}
