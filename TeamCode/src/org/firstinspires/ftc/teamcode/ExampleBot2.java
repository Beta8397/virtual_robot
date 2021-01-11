package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import system.robot.*;
import system.robot.localizer.HolonomicDriveEncoderIMULocalizer;
import system.robot.subsystems.drivetrain.DriveConfig;
import system.robot.subsystems.drivetrain.MecanumDriveSimple;

public class ExampleBot2 extends Robot {
    public MecanumDriveSimple drive;
    public ExampleBot2(OpMode opMode) {
        super(opMode);

        drive = new MecanumDriveSimple(this,
                new DriveConfig(2,1,15,1120,133.9),
                "front_left_motor",
                "front_right_motor",
                "back_left_motor",
                "back_right_motor", false
        );

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
