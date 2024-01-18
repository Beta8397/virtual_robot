package org.murraybridgebunyips.imposter.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.murraybridgebunyips.bunyipslib.*;
import org.murraybridgebunyips.bunyipslib.Scheduler;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.drive.CartesianMecanumDrive;
import org.murraybridgebunyips.bunyipslib.tasks.ContinuousTask;
import org.murraybridgebunyips.bunyipslib.tasks.HolonomicDriveTask;
import org.murraybridgebunyips.bunyipslib.tasks.InstantTask;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

@TeleOp(name = "TeleOp", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterTeleOpSimple extends BunyipsOpMode {
    private final ImposterConfig config = new ImposterConfig();
    private CartesianMecanumDrive drive;
    private Scheduler scheduler = new Scheduler(this);

    @Override
    protected void onInit() {
        config.init(this);
//        drive = new TriDeadwheelMecanumDrive(this, config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor, config.localizerCoefficients, config.enc_left, config.enc_right, config.enc_x);
        drive = new CartesianMecanumDrive(this, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor);
        scheduler.addSubsystems(drive);
        drive.setDefaultTask(new HolonomicDriveTask<>(gamepad1, drive));
        scheduler.when(() -> drive.speedX != 0.0 || drive.speedY != 0.0 || drive.speedR != 0.0)
                .runDebounced(new InstantTask(() -> log("Drive has started moving at % seconds.", getRuntime())))
                .immediately();
    }

    @Override
    protected void activeLoop() {
        scheduler.run();
    }
}
