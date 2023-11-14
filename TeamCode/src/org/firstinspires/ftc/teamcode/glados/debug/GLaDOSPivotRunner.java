package org.firstinspires.ftc.teamcode.glados.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.glados.components.GLaDOSConfigCore;

/**
 * Test pivot motor tracking and control using direct input
 *
 * @author Lucas Bubner, 2023
 */
@TeleOp(name = "GLaDOS: Rotator Motor Runner", group = "GLaDOS")
public class GLaDOSPivotRunner extends BunyipsOpMode {
    private GLaDOSConfigCore config = new GLaDOSConfigCore();

    @Override
    protected void onInit() {
        config = (GLaDOSConfigCore) RobotConfig.newConfig(this, config, hardwareMap);
    }

    @Override
    protected void activeLoop() {
        if (config.sr != null) {
            config.sr.setPower(gamepad1.left_stick_y);
            addTelemetry("Pivot Motor Position: %", config.sr.getCurrentPosition());
            addTelemetry("Pivot Motor Power: %", config.sr.getPower());
        }
    }
}
