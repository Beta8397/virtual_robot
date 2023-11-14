package org.firstinspires.ftc.teamcode.wheatley.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.wheatley.components.WheatleyConfig;

@TeleOp(name = "WHEATLEY: Rotation Test", group = "WHEATLEY")
public class WheatleyRotationTest extends BunyipsOpMode {
    private WheatleyConfig config = new WheatleyConfig();

    @Override
    protected void onInit() {
        config = (WheatleyConfig) RobotConfig.newConfig(this, config, hardwareMap);
    }

    @Override
    protected void activeLoop() {
        config.ra.setPower(gamepad1.left_stick_y);
    }
}
