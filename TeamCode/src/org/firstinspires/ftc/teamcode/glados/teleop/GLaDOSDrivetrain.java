package org.firstinspires.ftc.teamcode.glados.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.glados.components.GLaDOSConfigCore;
import org.firstinspires.ftc.teamcode.glados.components.GLaDOSPOVDriveCore;


/**
 * POV drivetrain only for GLaDOS.
 *
 * @author Lucas Bubner, 2023
 */
@TeleOp(name = "GLaDOS: Drivetrain", group = "GLaDOS")
public class GLaDOSDrivetrain extends BunyipsOpMode {
    private GLaDOSConfigCore config = new GLaDOSConfigCore();
    private GLaDOSPOVDriveCore drive;

    @Override
    protected void onInit() {
        config = (GLaDOSConfigCore) RobotConfig.newConfig(this, config, hardwareMap);
        drive = new GLaDOSPOVDriveCore(this, config.fl, config.bl, config.fr, config.br);
    }

    @Override
    protected void activeLoop() {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;
        drive.setSpeedUsingController(x, y, r / 2);
        drive.update();
    }
}

