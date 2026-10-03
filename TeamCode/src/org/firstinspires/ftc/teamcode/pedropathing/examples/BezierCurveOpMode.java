package org.firstinspires.ftc.teamcode.pedropathing.examples;


import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerLog;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.PedroTuningUtil;

import static com.pedropathing.api.Paths.curve;

@Autonomous(group = "PedroExamples")
public class BezierCurveOpMode extends LinearOpMode {
    double distance = 48;
    Follower follower;

    enum State {DRIVE1, TURN1, TURN2, DRIVE2}

    PoseFactory pf = PoseFactory.degrees();

    Pose pStart = pf.of(48, 48, 90);
    Pose pControl1 = pf.of(47, 80, 0);
    Pose pControl2 = pf.of(97, 65, 0);
    Pose pEnd = pf.of(96, 96, 90);
    Pose pEndTurned = pf.of(96, 96, 180);

    Path path1 = Paths.curve(pStart, pControl1, pControl2, pEnd).tangent();
    Path path2 = Paths.curve(pEnd, pControl2, pControl1, pStart).reverseTangent();

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(pStart);

        boolean forward = true;

        Thread.sleep(1000);
        waitForStart();
        follower.setPose(pStart);
        follower.update();
        follower.follow(path1);
        State state = State.DRIVE1;
        Pose targetPose = pEnd;
        boolean turning  = false;

        while (opModeIsActive() && !gamepad1.a) {
            follower.update();
            Pose pose = follower.pose();
            Velocity vel = follower.velocity();
            telemetry.addData("Pose", "x: %.1f  y: %.1f  h: %.1f",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("State", state);
            telemetry.update();

            if (!follower.isBusy()){
                switch (state){
                    case DRIVE1:
                        follower.hold(pEndTurned);
                        targetPose = pEndTurned;
                        state = State.TURN1;
                        turning = true;
                        break;
                    case TURN1:
                        follower.hold(pEnd);
                        targetPose = pEnd;
                        state = State.TURN2;
                        turning = true;
                        break;
                    case TURN2:
                        follower.follow(path2);
                        state = State.DRIVE2;
                        turning = false;
                        break;
                    case DRIVE2:
                        follower.follow(path1);
                        state = State.DRIVE1;
                        turning = false;
                }
            }

        }

        follower.drivetrain.stop(true);

        while (opModeIsActive()){
            continue;
        }
    }
}
