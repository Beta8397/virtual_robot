package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;
import system.robot.roadrunner_util.HALTrajectory;
import util.math.units.HALAngleUnit;

@StandAlone
@Autonomous(name = "Follower PID Tuner")
public class FollowerPIDTuner extends BaseAutonomous {
    public @MainRobot RoadrunnerCalibBot robot;
    private static final double DISTANCE = 48; // in
    private Pose2d startPose;

    @Override
    protected void onInit() {
        startPose = new Pose2d(DISTANCE / 2, -DISTANCE / 2, 0);

        robot.drive.setPoseEstimate(startPose);
    }

    @Override
    public void main() {
        while (!opModeIsActive()) {
            HALTrajectory traj = robot.drive.trajectoryBuilder(startPose)
                    .forward(DISTANCE)
                    .build();
            robot.drive.followTrajectory(traj);
            robot.drive.turn(90, HALAngleUnit.DEGREES);

            startPose = traj.end().plus(new Pose2d(0, 0, Math.toRadians(90)));
        }
    }
}
