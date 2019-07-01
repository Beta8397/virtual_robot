package teamcode.ftc16072;

import virtual_robot.controller.OpMode;

public class MecanumFieldRelativeOpMode extends OpMode {
    private Robot robot = new Robot();

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // mecanumDrive.driveMecanum(forward, strafe, rotate);
        robot.driveFieldRelative(telemetry, forward, strafe, rotate);
    }
}
