package teamcode.ftc16072;

import virtual_robot.annotations.TeleOp;
import virtual_robot.controller.OpMode;

@TeleOp(name = "mechanum driving opmode", group = "ftc16072")
public class MecanumDrivingOpMode extends OpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        mecanumDrive.driveMecanum(forward, strafe, rotate);

    }
}
