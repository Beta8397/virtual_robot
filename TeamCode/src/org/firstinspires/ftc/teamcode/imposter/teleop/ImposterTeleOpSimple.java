package org.firstinspires.ftc.teamcode.imposter.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.Controller;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.imposter.components.ImposterConfig;
import org.firstinspires.ftc.teamcode.imposter.components.ImposterDrive;

@TeleOp(name = "IMPOSTER: TeleOp", group = "IMPOSTER")
public class ImposterTeleOpSimple extends BunyipsOpMode {
    private ImposterConfig config = new ImposterConfig();
    private ImposterDrive drive;

    @Override
    protected void onInit() {
        config = (ImposterConfig) RobotConfig.newConfig(this, config, hardwareMap);
        drive = new ImposterDrive(this, config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.back_left_motor, config.front_right_motor, config.back_right_motor, config.localizerCoefficients, config.parallelEncoder, config.perpendicularEncoder);
    }

    @Override
    protected void activeLoop() {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;
        drive.setWeightedDrivePower(Controller.makeRobotPose(-x, -y, -r));
        drive.update();
    }
}
