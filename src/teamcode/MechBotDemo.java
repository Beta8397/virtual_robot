package teamcode;

import virtual_robot.controller.LinearOpMode;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.hardware.ColorSensor;
import virtual_robot.hardware.DCMotor;
import virtual_robot.hardware.GyroSensor;
import virtual_robot.hardware.Servo;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
public class MechBotDemo extends LinearOpMode {

    public void runOpMode(){
        DCMotor m1 = hardwareMap.dcMotor.get("back_left_motor");
        DCMotor m2 = hardwareMap.dcMotor.get("front_left_motor");
        DCMotor m3 = hardwareMap.dcMotor.get("front_right_motor");
        DCMotor m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DCMotor.Direction.REVERSE);
        m2.setDirection(DCMotor.Direction.REVERSE);
        m1.setMode(DCMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DCMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DCMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DCMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DCMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DCMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DCMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DCMotor.RunMode.RUN_USING_ENCODER);
        GyroSensor gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        Servo backServo = hardwareMap.servo.get("back_servo");
        gyro.init();
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            double px = gamePad1.left_stick_x;
            if (Math.abs(px) < 0.05) px = 0;
            double py = -gamePad1.left_stick_y;
            if (Math.abs(py) < 0.05) py = 0;
            double pa = -gamePad1.right_stick_x;
            if (Math.abs(pa) < 0.05) pa = 0;
            double p1 = -px + py - pa;
            double p2 = px + py + -pa;
            double p3 = -px + py + pa;
            double p4 = px + py + pa;
            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);
            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            telemetry.addData("Heading"," %.1f", gyro.getHeading());
            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
            telemetry.update();
        }
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}
