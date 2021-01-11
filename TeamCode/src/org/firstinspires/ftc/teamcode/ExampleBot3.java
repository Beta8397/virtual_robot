package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import system.robot.*;
import system.robot.localizer.NonHolonomicDriveEncoderIMULocalizer;
import system.robot.subsystems.drivetrain.DriveConfig;
import system.robot.subsystems.drivetrain.TankDriveSimple;

public class ExampleBot3 extends Robot {
    public TankDriveSimple drive;
    public ExampleBot3(OpMode opMode) {
        super(opMode);

        drive = new TankDriveSimple(
                this,
                new DriveConfig(2,1,15,1120,133.9),
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
