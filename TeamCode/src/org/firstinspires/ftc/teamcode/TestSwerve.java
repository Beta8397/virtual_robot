package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp(name = "Test Swerve", group = "Test")
public class TestSwerve extends OpMode {

    CRServo[] crServos = new CRServo[4];
    DcMotor[] encoders = new DcMotor[4];
    DcMotor[] motors = new DcMotor[4];

    public void init(){
        String[] crServoNames = new String[] {"back_left_crservo", "front_left_crservo", "front_right_crservo", "back_right_crservo"};
        for (int i=0; i<4; i++) crServos[i] = (CRServo)hardwareMap.get(CRServo.class, crServoNames[i]);
        String[] encoderNames = new String[] {"back_left_encoder", "front_left_encoder", "front_right_encoder", "back_right_encoder"};
        for (int i=0; i<4; i++) encoders[i] = (DcMotor)hardwareMap.get(DcMotor.class, encoderNames[i]);
        String[] motorNames = new String[] {"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (int i=0; i<4; i++) motors[i] = (DcMotor)hardwareMap.get(DcMotor.class, motorNames[i]);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        gamepad1.setJoystickDeadzone(0.05f);
    }

    public void loop(){
        if (gamepad1.a){
            crServos[0].setPower(0.1);
        } else if (gamepad1.x){
            crServos[1].setPower(0.1);
        } else if (gamepad1.y){
            crServos[2].setPower(0.1);
        } else if (gamepad1.b){
            crServos[3].setPower(0.1);
        } else {
            double pSteer = -gamepad1.right_stick_x;
            double pDrive = -gamepad1.left_stick_y;
            for (CRServo crservo : crServos) crservo.setPower(pSteer);
            for (DcMotor motor : motors) motor.setPower(pDrive);
        }
    }

}
