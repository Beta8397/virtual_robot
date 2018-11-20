package teamcode;

import hardware.DCMotor;
import hardware.GyroSensor;
import opmode.LinearOpMode;

public class ArcadeDrive extends LinearOpMode {

    public void runOpMode(){
        DCMotor left = hardwareMap.dcMotor.get("left_motor");
        DCMotor right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DCMotor.Direction.REVERSE);
        GyroSensor gyro = hardwareMap.gyroSensor.get("gyro_sensor");
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
