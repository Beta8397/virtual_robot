package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.lang.Math;

@Autonomous(name = "Test RR Auto", group = "RR")
public class TestRRAuto extends LinearOpMode {

    public class Arm{

        private DcMotorEx armMotor;
        private Servo clawServo;

        public Arm(HardwareMap hardwareMap){
            armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            clawServo = hardwareMap.get(Servo.class, "hand_servo");
        }

        public class SetArmPosition implements Action{
            private boolean initialized = false;
            private int targetPosition;

            public SetArmPosition(int targetPosition){
                this.targetPosition = targetPosition;
            }

            public boolean run(TelemetryPacket packet){
                if (!initialized){
                    initialized = true;
                    armMotor.setTargetPosition(targetPosition);
                    armMotor.setPower(1);
                }
                return armMotor.isBusy();
            }
        }
        public Action setPosition(int targetPosition){
            return new SetArmPosition(targetPosition);
        }

        public class SetClawPosition implements Action{
            double position;

            public SetClawPosition(double position){
                this.position = position;
            }

            public boolean run(TelemetryPacket packet){
                clawServo.setPosition(position);
                return false;
            }
        }
        public Action setClawPosition(double position){
            return new SetClawPosition(position);
        }
    }

    public void runOpMode(){

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-24, 24, 0));
        Arm arm = new Arm(hardwareMap);

        Action driveForward = drive.actionBuilder(new Pose2d(-24, 24, 0))
                        .setReversed(false)
                        .splineTo(new Vector2d(24, -24), 0)
                        .build();

        Action driveReverse = drive.actionBuilder(new Pose2d(24, -24,0))
                        .setReversed(true)
                        .splineTo(new Vector2d(-24, 24), Math.PI)
                        .build();


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        driveForward,
                        arm.setPosition(2000),
                        arm.setClawPosition(1),
                        driveReverse,
                        arm.setClawPosition(0),
                        arm.setPosition(0)
                )
        );

    }

}
