package org.firstinspires.ftc.teamcode.simple_rr_example;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "DriveSplineSimple", group = "road-runner")
public class DriveSplineSimple extends OpMode {

    MecBot bot;

    private enum State {BEGIN, AWAY, PAUSE, RETURN, DONE}
    State state = State.BEGIN;

    NanoClock clock;
    double startPause;

    Trajectory traj;

    public void init(){
        bot = new MecBot(hardwareMap);
        clock = NanoClock.system();
    }

    public void loop(){
        bot.updatePoseEstimate();
        Pose2d currentPose = bot.getPoseEstimate();

        telemetry.addData("STATE", state);
        telemetry.addData("POSE", "X = %.2f  Y = %.2f  H = %.1f", currentPose.getX(),
                currentPose.getY(), Math.toDegrees(currentPose.getHeading()));

        switch (state){
            case BEGIN:
                state = State.AWAY;
                traj = new TrajectoryBuilder(new Pose2d(), false, bot.velocityConstraint, bot.accelConstraint)
                        .splineTo(new Vector2d(30, 30), 0)
                        .build();
                bot.follower.followTrajectory(traj);
                break;
            case AWAY:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE;
                    bot.setDriveSignal(new DriveSignal());
                    startPause = clock.seconds();
                }
                break;
            case PAUSE:
                if ((clock.seconds() - startPause) > 2.0){
                    state = State.RETURN;
                    traj = new TrajectoryBuilder(new Pose2d(30, 30, 0), true, bot.velocityConstraint, bot.accelConstraint)
                            .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case RETURN:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.DONE;
                    bot.setDriveSignal(new DriveSignal());
                }
                break;
            case DONE:
                break;
        }
    }


}
