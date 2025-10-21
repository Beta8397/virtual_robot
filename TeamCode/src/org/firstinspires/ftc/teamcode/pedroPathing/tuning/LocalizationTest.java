package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot.
 * You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 *
 * Adapted by J. Kenney for use in virtual_robot.
 */
@TeleOp(group="Pedro")
public class LocalizationTest extends OpMode {

    Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    /** This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetry.addData("Press START when ready","");
        telemetry.addData("Drive with Gamepad1","");
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("heading:",
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("total heading:",
                Math.toDegrees(follower.getTotalHeading()));
    }
}
