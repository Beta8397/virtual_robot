package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import system.robot.*;
import system.robot.localizer.HolonomicDriveEncoderLocalizer;
import system.robot.roadrunner_util.RoadrunnerConfig;
import system.robot.subsystems.drivetrain.MecanumDrive;
import system.robot.subsystems.drivetrain.XDrive;

public class ExampleBot extends Robot {
    public MecanumDrive drive;
    public ExampleBot(OpMode opMode) {
        super(opMode);

        drive = new MecanumDrive(this,
                new RoadrunnerConfig(2,1,15,1120,133.9),
                "front_left_motor",
                "front_right_motor",
                "back_left_motor",
                "back_right_motor", false);

        /*drive.setLocalizer(new TwoWheelLocalizer(
                this,
                "imu",
                "enc_right",
                new Pose2d(0,-6,0),
                "enc_x",
                new Pose2d(0,0,PI/2),
                new TrackingWheelConfig(1,1,1120)
        ));*/
/*
        drive.setLocalizer(new ThreeWheelLocalizer(
                this,
                12,
                0,
                "enc_left",
                "enc_right",
                "enc_x",
                new TrackingWheelConfig(1,1,1120)
        ).reverseEncoder("enc_left"));
*/
        /*drive.setLocalizer(new DriveEncoderIMULocalizer(
                this,
                "imu",
                "front_left_motor",
                "front_right_motor",
                "back_left_motor",
                "back_right_motor",
                drive.driveConfig
        ));*/
        drive.setLocalizer(new HolonomicDriveEncoderLocalizer(
                drive,
                "front_left_motor",
                "front_right_motor",
                "back_left_motor",
                "back_right_motor"
        ));

    }
}
