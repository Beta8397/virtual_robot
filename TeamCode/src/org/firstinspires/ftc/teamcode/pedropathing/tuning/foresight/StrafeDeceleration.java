package org.firstinspires.ftc.teamcode.pedropathing.tuning.foresight;


import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.TuningOpMode;

import java.util.ArrayList;

@Disabled
@Autonomous(group = "PedroTune")
public class StrafeDeceleration extends TuningOpMode {
    double velocity = 48;

    private final ArrayList<Double> accelerations = new ArrayList<>();

    private double previousVelocity;
    private long previousTimeNano;
    private boolean stopping;

    @Override
    public void runTuningOpMode() throws InterruptedException {

        accelerations.clear();
        previousVelocity = 0;
        previousTimeNano = 0;
        stopping = false;

        localizer.setPose(Pose.zero());
        localizer.update();

        DrivePowers power = new DrivePowers(0, 1, 0);
        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();

        drivetrain.drive(power, false);

        while (opModeIsActive() && !stopping) {
            localizer.update();
            double currentVelocity = localizer.twist().toVector2D().y();
            if (Math.abs(currentVelocity) > velocity) {
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();

                stopping = true;
                drivetrain.stop(false);
            } else {
                drivetrain.drive(power, false);
            }
        }

        double startVelocity = localizer.twist().toVector2D().y();
        long startNanos = System.nanoTime();
        double result = 0;
        boolean end = false;

        while (opModeIsActive() && !end) {
            drivetrain.stop(false);
            localizer.update();
            double currentVelocity = localizer.twist().toVector2D().y();
            long currentTimeNano = System.nanoTime();
//            double dt = (currentTimeNano - previousTimeNano) / 1e9;
//
//            if (dt > 0) {
//                double acceleration = (currentVelocity - previousVelocity) / dt;
//                accelerations.add(acceleration);
//            }
//
//            previousVelocity = currentVelocity;
//            previousTimeNano = currentTimeNano;


            if (Math.abs(currentVelocity) <= 1) {
                end = true;
                result = (currentVelocity-startVelocity)*1.0e9/(currentTimeNano-startNanos);
                System.out.printf("\nstartVel: %.2f  endVel: %.2f  startNano: %d  endNano: %d",
                        startVelocity, currentVelocity, startNanos, currentTimeNano);
            }

        }

        drivetrain.stop(false);

//        double average = 0;
//
//        for (double acceleration : accelerations) {
//            average += acceleration;
//        }
//
//        double result = accelerations.isEmpty()? 0.0 : Math.abs(average / accelerations.size());

        while (opModeIsActive()){
            telemetry.addData("Strafe Decel", "%.5f", result);
            telemetry.update();
        }
    }
}
