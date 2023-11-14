package org.firstinspires.ftc.teamcode.glados.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.*;
import org.firstinspires.ftc.teamcode.glados.components.*;

/**
 * TeleOp for GLaDOS robot FTC 15215
 * > gamepad1.left_stick for planar translation
 * > gamepad1.right_stick for in-place rotation
 * > gamepad1.left_bumper to decrement mecanum speed
 * > gamepad1.right_bumper to increment mecanum speed
 * > gamepad2.left_stick_y for linear slide rotation
 * > gamepad2.right_stick_y for linear slide extension
 * > gamepad2.dpad_up to align linear slide upwards
 * > gamepad2.dpad_down to align linear slide downwards
 * > gamepad2.x to toggle left claw
 * > gamepad2.b to toggle right claw
 * > gamepad2.right_trigger to full to fire cannon
 * > gamepad2.back to reset cannon
 *
 * @author Lucas Bubner, 2023
 */
@TeleOp(name = "GLaDOS: TeleOp", group = "GLaDOS")
public class GLaDOSTeleOp extends BunyipsOpMode {
    private final TriSpeed speed = new TriSpeed(TriSpeed.Speed.NORMAL);
    private GLaDOSConfigCore config = new GLaDOSConfigCore();
    private GLaDOSPOVDriveCore drive;
    private GLaDOSArmCore arm;
    private Cannon cannon;
    private boolean x_pressed;
    private boolean b_pressed;
    private boolean inc_pressed;
    private boolean dec_pressed;

    @Override
    protected void onInit() {
        config = (GLaDOSConfigCore) RobotConfig.newConfig(this, config, hardwareMap);
        drive = new GLaDOSPOVDriveCore(this, config.fl, config.bl, config.fr, config.br);
        arm = new GLaDOSArmCore(this, config.sr, config.sa, config.al, config.ls, config.rs, GLaDOSAlignmentCore.Mode.MANUAL);
        if (NullSafety.assertComponentArgs(this, Cannon.class, config.pl))
            cannon = new Cannon(this, config.pl);
    }

    @Override
    protected void activeLoop() {
        // Mecanum drive
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;

        if (gamepad1.right_bumper && !inc_pressed) {
            speed.increment();
        } else if (gamepad1.left_bumper && !dec_pressed) {
            speed.decrement();
        }
        addTelemetry("TriSpeed: Running at % speed", speed.getSpeed());

        drive.setSpeedUsingController(x * speed.getMultiplier(), y * speed.getMultiplier(), r * speed.getMultiplier());

        // Linear slider rotator
        arm.getSliderController().setTargetAngleUsingController(gamepad2.left_stick_y);
        // Linear slider extender
        arm.getSliderController().setExtrusionPowerUsingController(gamepad2.right_stick_y);
        // Linear slider alignment servo
        arm.getAlignmentController().setPositionUsingDpad(gamepad2.dpad_up, gamepad2.dpad_down);

        // Left claw servo
        if (gamepad2.x && !x_pressed) {
            arm.getServoController().toggleServo(GLaDOSServoCore.ServoSide.LEFT);
        }

        // Right claw servo
        if (gamepad2.b && !b_pressed) {
            arm.getServoController().toggleServo(GLaDOSServoCore.ServoSide.RIGHT);
        }

        // Airplane launcher
        if (gamepad2.right_trigger == 1.0) {
            cannon.fire();
        }
        if (gamepad2.back) {
            cannon.reset();
        }

        // Ensure that the buttons are only registered once per press
        x_pressed = gamepad2.x;
        b_pressed = gamepad2.b;
        inc_pressed = gamepad1.right_bumper;
        dec_pressed = gamepad1.left_bumper;

        // Dispatch stateful changes
        drive.update();
        arm.update();
        cannon.update();
    }
}
