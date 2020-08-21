package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TestProgrammingBoard", group = "Test")
public class TestProgrammingBoard extends OpMode {

    DcMotor motor = null;
    Servo servo = null;
    BNO055IMU imu = null;
    ColorSensor colorSensor = null;
    AnalogInput analogInput = null;
    DigitalChannel digitalChannel = null;

    ElapsedTime et = null;
    double power = 0.2;
    double servoPos = 0;

    public void init(){
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = hardwareMap.get(Servo.class, "servo");
        servoPos = 0;
        servo.setPosition(servoPos);

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        analogInput = hardwareMap.get(AnalogInput.class, "pot");
        digitalChannel = hardwareMap.get(DigitalChannel.class, "touch_sensor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu.initialize(parameters);

        et = new ElapsedTime();
    }

    public void loop(){
        if (et.seconds() >= 2){
            et.reset();
            power *= -1;
            servoPos = 1 - servoPos;
        }

        motor.setPower(power);
        servo.setPosition(servoPos);
        Orientation orientation = imu.getAngularOrientation();
        telemetry.addData("Heading", "%.1f deg", orientation.firstAngle);
        telemetry.addData("Ticks", motor.getCurrentPosition());
        telemetry.addData("Volts", analogInput.getVoltage());
        telemetry.addData("Touch", !digitalChannel.getState());
        telemetry.addData("Color", "R: %d  G: %d  B: %d", colorSensor.red(),
                colorSensor.green(), colorSensor.blue());

    }

}
