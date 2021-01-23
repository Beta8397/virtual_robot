package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import system.gui.menus.examplemenu.ExampleMenu;
import system.robot.Robot;
import system.robot.localizer.*;
import system.robot.roadrunner_util.CoordinateMode;
import system.robot.roadrunner_util.RoadrunnerConfig;
import system.robot.subsystems.drivetrain.MecanumDrive;
import util.control.Button;

import static java.lang.Math.PI;

public class RoadrunnerCalibBot extends Robot {
    public MecanumDrive drive;
    public RoadrunnerCalibBot(OpMode opMode) {
        super(opMode);

        drive = new MecanumDrive(this,
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
