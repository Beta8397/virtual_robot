package teamcode;

import virtual_robot.hardware.ColorSensor;
import virtual_robot.hardware.DCMotor;
import virtual_robot.hardware.GyroSensor;
import virtual_robot.hardware.Servo;
import virtual_robot.controller.LinearOpMode;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
public class TwoWheelDemo extends LinearOpMode {

    public void runOpMode(){
        DCMotor left = hardwareMap.dcMotor.get("left_motor");
        DCMotor right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DCMotor.Direction.REVERSE);
        GyroSensor gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        Servo backServo = hardwareMap.servo.get("back_servo");
        gyro.init();
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            if (gamePad1.a){
                telemetry.addData("a pressed","");
                left.setPower(-.5);
                right.setPower(-.5);
            } else if (gamePad1.y) {
                telemetry.addData("y pressed", "");
                left.setPower(0.5);
                right.setPower(0.5);
            } else if (gamePad1.b){
                telemetry.addData("b pressed", "");
                left.setPower(0.5);
                right.setPower(-0.5);
            } else if (gamePad1.x){
                telemetry.addData("x pressed", "");
                left.setPower(-0.5);
                right.setPower(0.5);
            }
            else {
                left.setPower(0);
                right.setPower(0);
            }
            backServo.setPosition(0.5 - 0.5*gamePad1.left_stick_y);
            telemetry.addData("Press", "Y-fwd, A-rev, B-Rt, X-Lt");
            telemetry.addData("Left Gamepad stick controls back servo","");
            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            telemetry.addData("Heading"," %.1f", gyro.getHeading());
            telemetry.addData("Encoders","Left %d  Right %d", left.getCurrentPosition(), right.getCurrentPosition());
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }
}
