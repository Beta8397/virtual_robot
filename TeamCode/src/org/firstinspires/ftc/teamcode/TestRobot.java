package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import system.robot.Robot;
import system.robot.localizer.HolonomicDriveEncoderIMULocalizer;
import system.robot.roadrunner_util.RoadrunnerConfig;
import system.robot.subsystems.drivetrain.MecanumDrive;
import system.robot.subsystems.drivetrain.MecanumDriveSimple;

public class TestRobot extends Robot {
    public MecanumDriveSimple drive;
    public TestRobot(OpMode opMode) {
        super(opMode);

        drive = new MecanumDriveSimple(this,
                new RoadrunnerConfig(2,1,15,1120,133.9),
                "front_left_motor",
                "front_right_motor",
                "back_left_motor",
                "back_right_motor", false);
        drive.setLocalizer(new HolonomicDriveEncoderIMULocalizer(
                this,
                drive,
                "imu",
                "front_left_motor",
                "front_right_motor",
                "back_left_motor",
                "back_right_motor"
        ));
    }
}
