package org.firstinspires.ftc.teamcode.pedropathing.tuning.foresight;


import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.TuningOpMode;

import java.util.ArrayDeque;

@Disabled
@Autonomous(group = "PedroTune")
public class ForwardVelocity extends TuningOpMode {
    double distance = 48;
    private final ArrayDeque<Double> velocities = new ArrayDeque<>();
    public static double RECORD_NUMBER = 10;


    @Override
    public void runTuningOpMode() throws InterruptedException {

        System.out.println("runTuningOpMode");
        boolean end = false;

        localizer.setPose(Pose.zero());
        localizer.update();

        DrivePowers power = new DrivePowers(1,0,0);

        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();

        while (opModeIsActive() && !end) {
            localizer.update();
            if (Math.abs(localizer.pose().x()) > distance) {
                end = true;
                drivetrain.stop();
            } else {
                drivetrain.drive(power, true);
                double currentVelocity = Math.abs(localizer.twist().toVector2D().x());
                velocities.addLast(currentVelocity);
                velocities.removeFirst();
            }
        }

        drivetrain.stop();
        double average = 0;
        for (double velocity : velocities) {
            average += velocity;
        }
        average /= velocities.size();

        while (opModeIsActive()){
            telemetry.addData("Max Fwd Vel", "%.2f", average);
            telemetry.update();
        }
    }
}
