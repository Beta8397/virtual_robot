package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ftc16072.Util.Polar;

@TeleOp(name = "0 - TurretOpMode", group = "ftc16072")
public class TurretOpMode extends OpMode {
    private Robot robot = new Robot();
    Servo turretServo, elevationServo;
    boolean wasLeft, wasRight, wasDown, wasUp;
    double turretPos;
    double elevationPos;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
        turretServo = hardwareMap.get(Servo.class, "turret_servo");
        elevationServo = hardwareMap.get(Servo.class, "elevation_servo");

        turretPos = 0.5;
        elevationPos = 0;
        turretServo.setPosition(turretPos);
        elevationServo.setPosition(elevationPos);
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        double strafe = gamepad1.left_stick_x;

        Polar g1RightJoystick = Polar.fromCartesian(gamepad1.right_stick_x, -gamepad1.right_stick_y);

        double r = g1RightJoystick.getR();
        if (r >= 0.8) {
            robot.nav.driveFieldRelativeAngle(strafe, forward, g1RightJoystick.getTheta());
        } else {
            robot.nav.driveFieldRelative(strafe, forward, 0.0);
        }

        if (gamepad1.dpad_up && !wasUp) {
            elevationPos = Math.min(elevationPos + .05, 1);
        }
        wasUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !wasDown) {
            elevationPos = Math.max(elevationPos - 0.05, 0);
        }
        wasDown = gamepad1.dpad_down;

        if (gamepad1.dpad_left && !wasLeft) {
            turretPos = Math.max(turretPos - .05, 0);
        }
        wasLeft = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !wasRight) {
            turretPos = Math.min(turretPos + 0.05, 1);
        }
        wasRight = gamepad1.dpad_right;

        turretServo.setPosition(turretPos);
        elevationServo.setPosition(elevationPos);
    }
}

