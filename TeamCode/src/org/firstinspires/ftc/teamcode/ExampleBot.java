package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import system.robot.Robot;
import system.robot.subsystems.MechanumDrive;

public class ExampleBot extends Robot {
    public MechanumDrive drive;
    public ExampleBot(OpMode opMode) {
        super(opMode);

        drive = new MechanumDrive(this,
                "front_left_motor",
                "front_right_motor",
                "back_left_motor",
                "back_right_motor");
    }
}
