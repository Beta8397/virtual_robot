package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.tuning_util.Adjuster;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "Pedro")
@Disabled
public class HeadingTuner extends OpMode {

    Follower follower;
    List<Adjuster> adjusters;
    Adjuster adjuster;
    double targetH = 0;

    public void init(){
        follower = Constants.createFollower(hardwareMap);
        adjusters = new ArrayList<>();
        adjusters.add(new Adjuster("P", ()->follower.getConstants().getCoefficientsHeadingPIDF().P,
                (double p)->follower.getConstants().getCoefficientsHeadingPIDF().P = p, gamepad1));
        adjusters.add(new Adjuster("D", ()->follower.getConstants().getCoefficientsHeadingPIDF().D,
                (double d)->follower.getConstants().getCoefficientsHeadingPIDF().D = d, gamepad1));
        adjusters.add(new Adjuster("F", ()->follower.getConstants().getCoefficientsHeadingPIDF().F,
                (double f)->follower.getConstants().getCoefficientsHeadingPIDF().F = f, gamepad1));
        adjuster = adjusters.get(0);
    }

    public void start(){
        follower.setPose(new Pose(0,0,0));
        follower.holdPoint(new Pose(0,0,0));
    }

    public void loop(){
        Pose pose = follower.getPose();
        if (gamepad1.leftBumperWasReleased()){
            targetH = 0;
            follower.holdPoint(new Pose(0, 0, 0));
        } else if (gamepad1.left_bumper){
            targetH += Math.toRadians(1);
            follower.holdPoint(new Pose(0, 0, targetH));
        }
        follower.update();

        if (gamepad1.rightBumperWasPressed()){
            adjuster = adjusters.get((adjusters.indexOf(adjuster)+1)%adjusters.size());
        }

        double val = adjuster.update();
        PIDFCoefficients pidf = follower.getConstants().getCoefficientsTranslationalPIDF();
        telemetry.addData("PIDF", "%s = %.6f", adjuster.getName(), val);
        telemetry.addData("Lt Bumper to disturb robot","");
        telemetry.addData("Rt bumper for next parameter","");
        telemetry.addData("Dpad to change parameter value","");
    }
}
