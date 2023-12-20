package org.firstinspires.ftc.teamcode.imposter.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.Controller;
import org.firstinspires.ftc.teamcode.common.DualDeadwheelMecanumDrive;
import org.firstinspires.ftc.teamcode.imposter.components.ImposterConfig;

@TeleOp(name = "TeleOp", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterTeleOpSimple extends BunyipsOpMode {
    private final ImposterConfig config = new ImposterConfig();
    private DualDeadwheelMecanumDrive drive;

    @Override
    protected void onInit() {
        config.init(this);
        drive = new DualDeadwheelMecanumDrive(this, config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor, config.localizerCoefficients, config.parallelEncoder, config.perpendicularEncoder);
    }

    @Override
    protected void activeLoop() {
        drive.setSpeedUsingController(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        drive.update();
    }
}
