package teamcode;

import hardware.DCMotor;
import hardware.GyroSensor;
import opmode.LinearOpMode;

public class TestOpMode2 extends LinearOpMode {

    public void runOpMode(){
        DCMotor left = hardwareMap.dcMotor.get("left_motor");
        DCMotor right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DCMotor.Direction.REVERSE);
        GyroSensor gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        waitForStart();
        while (opModeIsActive()){
            if (gamePad1.a){
                telemetry.addData("a pressed","");
                left.setPower(.5);
                right.setPower(.5);
            } else if (gamePad1.y){
                telemetry.addData("b pressed", "");
                left.setPower(-.5);
                right.setPower(.5);
            } else {
                left.setPower(0);
                right.setPower(0);
            }
            telemetry.addData("Heading", " %.1f", gyro.getHeading());
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }
}
