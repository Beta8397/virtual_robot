package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.MovingStatistics;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.MainRobot;

@StandAlone
@Autonomous(name = "Track Width Tuner")
public class TrackWidthTuner extends BaseAutonomous {
    public @MainRobot RoadrunnerCalibBot robot;

    public static double ANGLE = 90; // deg
    public static int NUM_TRIALS = 5;
    public static int DELAY = 1000; // ms

    @Override
    protected void onInit() {
        robot.telemetry.addData("","Press play to begin the track width tuner routine");
        robot.telemetry.addData("","Make sure your robot has enough clearance to turn smoothly");
        robot.telemetry.update();

    }

    @Override
    public void main() {
        robot.telemetry.addData("","Running...");
        robot.telemetry.update();

        MovingStatistics trackWidthStats = new MovingStatistics(NUM_TRIALS);
        for (int i = 0; i < NUM_TRIALS; i++) {
            robot.drive.getLocalizer().setPoseEstimate(new Pose2d());

            // it is important to handle heading wraparounds
            double headingAccumulator = 0;
            double lastHeading = 0;

            robot.drive.turnAsync(Math.toRadians(ANGLE));

            while (!isStopRequested() && robot.drive.rRInterfaceIsBusy()) {
                double heading = robot.drive.getPoseEstimate().getHeading();
                headingAccumulator += Angle.norm(heading - lastHeading);
                lastHeading = heading;

                robot.drive.updateRRInterface();
            }

            double trackWidth = robot.drive.driveConfig.TRACK_WIDTH * Math.toRadians(ANGLE) / headingAccumulator;
            trackWidthStats.add(trackWidth);

            waitTime(DELAY);
        }

        robot.telemetry.addData("","Tuning complete");
        robot.telemetry.addData("","Effective track width = "+trackWidthStats.getMean()+" (SE = "+trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)+")");
        robot.telemetry.update();

        waitUntil(()->robot.isStopRequested());
    }
}
