package org.murraybridgebunyips.imposter.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.drive.CartesianMecanumDrive;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

@TeleOp(name = "Standard Drive", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterDrive extends BunyipsOpMode {
    private final ImposterConfig config = new ImposterConfig();
    private CartesianMecanumDrive drive;

    @Override
    protected void onInit() {
        config.init(this);
        drive = new CartesianMecanumDrive(this, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor);
    }

    @Override
    protected void activeLoop() {
        drive.setSpeedUsingController(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        drive.update();
    }
}
