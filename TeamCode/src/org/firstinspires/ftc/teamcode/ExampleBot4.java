package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import system.robot.*;
import system.robot.localizer.NonHolonomicDriveEncoderIMULocalizer;
import system.robot.roadrunner_util.RoadrunnerConfig;
import system.robot.subsystems.drivetrain.TankDrive;

public class ExampleBot4 extends Robot {
    public TankDrive drive;
    public ExampleBot4(OpMode opMode) {
        super(opMode);

        drive = new TankDrive(
                this,
                new RoadrunnerConfig(2,1,15,1120,133.9),
                "left_motor",
                "right_motor"
        );
        drive.setLocalizer(new NonHolonomicDriveEncoderIMULocalizer(
                this,
                drive,
                "imu",
                "left_motor",
                "right_motor"
        ));
    }
}
