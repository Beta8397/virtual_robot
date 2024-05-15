package org.murraybridgebunyips.imposter.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.drive.CartesianMecanumDrive;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

import static org.murraybridgebunyips.bunyipslib.Text.round;

@TeleOp(name = "Standard Drive", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterDrive extends BunyipsOpMode {
    private final ImposterConfig config = new ImposterConfig();
    private CartesianMecanumDrive drive;

    @Override
    protected void onInit() {
        config.init();
        drive = new CartesianMecanumDrive(config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor);
    }

    @Override
    protected void activeLoop() {
        drive.setSpeedUsingController(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        drive.update();
        telemetry.add("actual speeds: fl:%, fr:%, bl:%, br:%",
                round(config.front_left_motor.getPower(), 3, 4),
                round(config.front_right_motor.getPower(), 3, 4),
                round(config.back_left_motor.getPower(), 3, 4),
                round(config.back_right_motor.getPower(), 3,4)
        );
    }
}
