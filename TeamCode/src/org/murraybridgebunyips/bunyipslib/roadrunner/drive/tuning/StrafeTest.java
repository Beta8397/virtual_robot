package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
//@Config
public abstract class StrafeTest extends LinearOpMode {
    public static double DISTANCE = 60; // in
    protected RoadRunnerDrive drive;

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        if (drive == null) throw new NullPointerException("drive is null!");


        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}