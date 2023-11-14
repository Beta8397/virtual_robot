package org.firstinspires.ftc.teamcode.wheatley.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.Cannon;
import org.firstinspires.ftc.teamcode.common.NullSafety;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.wheatley.components.WheatleyConfig;
import org.firstinspires.ftc.teamcode.wheatley.components.WheatleyLift;
import org.firstinspires.ftc.teamcode.wheatley.components.WheatleyManagementRail;
import org.firstinspires.ftc.teamcode.wheatley.components.WheatleyMecanumDrive;

/**
 * Primary TeleOp for all of Wheatley's functions.
 * Uses gamepad1 for drive control (presumably) and gamepad2 for both arms/claws
 * > gamepad1 left stick for driving
 * > gamepad1 right stick for turning
 * > gamepad2 left stick for Pixel Claw arm
 * > gamepad2 right stick for Suspender arm (only works when arm is released)
 * > gamepad2 dpad up releases the suspender
 * > gamepad1 right trigger fires paper plane
 * > gamepad2 x operates the left pixel claw
 * > gamepad2 y operates the right pixel claw
 *
 * @author Lachlan Paul, 2023
 * @author Lucas Bubner, 2023
 */

@TeleOp(name = "WHEATLEY: TeleOp", group = "WHEATLEY")
public class WheatleyTeleOp extends BunyipsOpMode {

    private WheatleyConfig config = new WheatleyConfig();
    private WheatleyMecanumDrive drive;
    private WheatleyLift lift;
    private WheatleyManagementRail suspender; // no way it's the wheatley management rail:tm:
    private Cannon cannon;

    private boolean xPressed;
    private boolean yPressed;

    @Override
    protected void onInit() {
        config = (WheatleyConfig) RobotConfig.newConfig(this, config, hardwareMap);
        drive = new WheatleyMecanumDrive(this, config.fl, config.bl, config.fr, config.br);
        if (NullSafety.assertComponentArgs(this, WheatleyLift.class, config.ra, config.ls, config.rs))
            lift = new WheatleyLift(this, config.ra, config.ls, config.rs);
        if (NullSafety.assertComponentArgs(this, WheatleyManagementRail.class, config.susMotor, config.susServo))
            suspender = new WheatleyManagementRail(this, config.susMotor, config.susServo);
        if (NullSafety.assertComponentArgs(this, Cannon.class, config.pl))
            cannon = new Cannon(this, config.pl);

        // TODO: Review systems for operator ease of use
    }

    @Override
    protected void activeLoop() {
        drive.setSpeedUsingController(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        lift.actuateUsingController(gamepad2.left_stick_y);

        // Launches the paper plane
        // The triggers are pressure sensitive, apparently.
        // Set to 1 to avoid any slight touches launching a nuke.
        if (gamepad2.right_trigger == 1.0) {
            // "Now then, let's see what we got here. Ah! 'Reactor Core Emergency Heat Venting Protocols.'
            // that's the problem right there, isn't it? 'Emergency'. you don't want to see 'emergency'
            // flashing at you. Never good that, is it? Right. DELETE."
            cannon.fire();
        }

        // Reset cannon for debugging purposes
        if (gamepad2.options) {
            // "Undelete, undelete! Where's the undelete button?"
            cannon.reset();
        }

        // Unlock suspender
        if (gamepad2.dpad_up) {
            suspender.activate();
        }
        // Will noop if the suspender is locked
        suspender.actuateUsingController(gamepad2.right_stick_y);

        // Claw controls
        if (gamepad2.x && !xPressed) {
            lift.toggleLeftClaw();
        } else if (gamepad2.y && !yPressed) {
            lift.toggleRightClaw();
        }

        // Register actions only once per press
        xPressed = gamepad2.x;
        yPressed = gamepad2.y;

        /*
         * TODO: Pick out some good Wheatley voice lines for telemetry
         * Different lines will be displayed depending on different values
         * They should overwrite each other and NOT stack
         *
         * They should also be out of the way of the other actually important telemetry
         * "Could a moron do THAT?" as wheatley throws 35,000 errors at once
         */

        // Send stateful updates to the hardware
        drive.update();
        lift.update();
        suspender.update();
        cannon.update();
    }
}