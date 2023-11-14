package org.firstinspires.ftc.teamcode.glados.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.glados.components.GLaDOSConfigCore;

import java.util.Locale;

/**
 * Test arm rotation tracking and control
 *
 * @author Lucas Bubner, 2023
 */
@TeleOp(name = "GLaDOS: Rotator Motor Degrees Runner", group = "GLaDOS")
public class GLaDOSRotateTest extends BunyipsOpMode {
    double target;
    private GLaDOSConfigCore config = new GLaDOSConfigCore();

    @Override
    protected void onInit() {
        config = (GLaDOSConfigCore) RobotConfig.newConfig(this, config, hardwareMap);
        config.sr.reset();
        config.sr.track();
        config.sr.setTargetPosition(0);
        config.sr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        config.sr.setPower(1);
    }

    @Override
    protected void activeLoop() {
        target -= gamepad1.left_stick_y / 2;
        addTelemetry("Degrees: %", String.format(Locale.getDefault(), "%.2f", config.sr.getDegrees()));
        config.sr.setDegrees(target);
    }
}
