package org.firstinspires.ftc.teamcode.pedropathing.examples;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.pedro.PedroCommands;
import static com.pedropathing.ivy.commands.Commands.*;
import com.pedropathing.ivy.groups.Groups;
import static com.pedropathing.ivy.groups.Groups.*;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.Constants;

@Autonomous(group = "PedroExamples")
public class IvyOpMode extends OpMode {

    Follower follower;
    DcMotorEx armMotor;
    Servo handServo;

    PoseFactory pf = PoseFactory.degrees();
    Pose pStart = pf.of(48, 48, 90);
    Pose pControl1 = pf.of(47, 80, 0);
    Pose pControl2 = pf.of(97, 65, 0);
    Pose pEnd = pf.of(96, 96, 90);

    Path path1 = Paths.curve(pStart, pControl1, pControl2, pEnd).tangent();
    Path path2 = Paths.curve(pEnd, pControl2, pControl1, pStart).tangent();

    Command setArm(int pos) {
        return sequential(
                instant(()->{armMotor.setTargetPosition(pos); armMotor.setPower(1);}),
                waitUntil(()->Math.abs(pos - armMotor.getCurrentPosition()) < 10)
        );
    }

    Command setServo(double pos){
        return sequential(instant(()->handServo.setPosition(pos)), waitMs(500));
    }

    Command waitForFollower(double p) {
        return waitUntil(() -> follower.completion() > p);
    }

    Command drivePath(Path path) {
        return PedroCommands.follow(follower, path);
    }

    Command mainCommand() {
        return Groups.loop(sequential(
                parallel(drivePath(path1), sequential(waitForFollower(0.5), setArm(2000))),
                setServo(0),
                setArm(0),
                parallel(drivePath(path2), sequential(waitForFollower(0.5), setArm(2000))),
                setServo(1),
                setArm(0)
        ));
    }


    public void init(){
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        handServo = hardwareMap.get(Servo.class, "hand_servo");
        handServo.setPosition(1);
        Scheduler.schedule(mainCommand());
    }

    public void start(){
        follower.setPose(pStart);
        follower.update();
    }

    public void loop(){
        follower.update();
        Scheduler.execute();
    }
}
