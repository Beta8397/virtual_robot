package org.murraybridgebunyips.imposter.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.NullSafety;
import org.murraybridgebunyips.bunyipslib.TriDeadwheelMecanumDrive;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

import java.util.ArrayList;

@TeleOp(name = "TeleOp", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterTeleOpSimple extends BunyipsOpMode {
    private final ImposterConfig config = new ImposterConfig();
    private MecanumDrive drive;

    @Override
    protected void onInit() {
        config.init(this);
        drive = new TriDeadwheelMecanumDrive(this, config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor, config.localizerCoefficients, config.enc_left, config.enc_right, config.enc_x);
    }

    @Override
    protected void activeLoop() {
        drive.setSpeedUsingController(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        drive.update();
    }
}
