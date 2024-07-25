package org.murraybridgebunyips.imposter.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Reference;
import org.murraybridgebunyips.bunyipslib.drive.CartesianMecanumDrive;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

import static org.murraybridgebunyips.bunyipslib.Text.round;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

@TeleOp
public class ImposterSlipperyDrive extends BunyipsOpMode {
    private final ImposterConfig config = new ImposterConfig();
    private CartesianMecanumDrive d;

    private final Reference<Double> x = Reference.empty(), y = Reference.empty(), r = Reference.empty();

    @Override
    protected void onInit() {
        config.init();
        d = new CartesianMecanumDrive(config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor);
    }

    @Override
    protected void activeLoop() {
        d.speedX = Mathf.smoothDamp(d.speedX, gamepad1.lsx, x, Seconds.of(1), 0.1, timer.deltaTime());
        d.speedY = Mathf.smoothDamp(d.speedY, -gamepad1.lsy, y, Seconds.of(1), 0.1, timer.deltaTime());
        d.speedR = Mathf.smoothDamp(d.speedR, gamepad1.rsx, r, Seconds.of(1), 0.1, timer.deltaTime());
        telemetry.add("velocities -> x:% y:% r:% %", round(x.get(), 3), round(y.get(), 3), round(r.get(), 3));
        d.update();
    }

}
