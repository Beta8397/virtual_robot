package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Math.PI;

@Autonomous(name = "Test Auto", group = "Test")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory t = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30,-30),0)
                .build();
        drive.followTrajectory(t);

        Trajectory a = drive.trajectoryBuilder(t.end(), true)
                .splineTo(new Vector2d(0,0), PI)
                .build();
        drive.followTrajectory(a);
    }
}
