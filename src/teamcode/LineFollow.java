package teamcode;

import virtual_robot.hardware.ColorSensor;
import virtual_robot.hardware.DCMotor;
import javafx.scene.paint.Color;
import virtual_robot.opmode.LinearOpMode;

/**
 * Example OpMode. Demonstrates autonomous proportionate line following.
 */
public class LineFollow extends LinearOpMode {

    public void runOpMode(){
        DCMotor left = hardwareMap.dcMotor.get("left_motor");
        DCMotor right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DCMotor.Direction.REVERSE);
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        waitForStart();
        double speed = 0.25;
        while (opModeIsActive()){
            Color c = Color.rgb(colorSensor.red(), colorSensor.green(), colorSensor.blue());
            double sat = c.getSaturation();
            double steer = 0.2 * (sat - 0.5);
            double vLeft = speed * (1.0 + steer);
            double vRight = speed * (1.0 - steer);
            left.setPower(vLeft);
            right.setPower(vRight);
            }
        left.setPower(0);
        right.setPower(0);
    }
}
