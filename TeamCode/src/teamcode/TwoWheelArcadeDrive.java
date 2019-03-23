package teamcode;

import virtual_robot.hardware.DCMotor;
import virtual_robot.controller.LinearOpMode;

/**
 * Example OpMode. Controls robot using left joystick, with arcade drive.
 */
public class TwoWheelArcadeDrive extends LinearOpMode {

    public void runOpMode(){
        DCMotor left = hardwareMap.dcMotor.get("left_motor");
        DCMotor right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DCMotor.Direction.REVERSE);
        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            float fwd = -gamePad1.left_stick_y;
            float steer = gamePad1.left_stick_x;
            left.setPower(.5 * fwd + .2 * steer);
            right.setPower(0.5 * fwd - .2 * steer);
            }
        left.setPower(0);
        right.setPower(0);
    }
}
