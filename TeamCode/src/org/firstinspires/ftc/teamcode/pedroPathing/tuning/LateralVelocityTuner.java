package org.firstinspires.ftc.teamcode.pedroPathing.tuning;


import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

/**
 * This is the ForwardVelocityTuner autonomous follower OpMode. This runs the robot forwards at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with StrafeVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 *
 * Adapted by J. Kenney for use in virtual_robot.
 */
@Autonomous(group="Pedro")
@Disabled
public class LateralVelocityTuner extends OpMode {

    Follower follower;

    private final ArrayList<Double> velocities = new ArrayList<>();
    public static double DISTANCE = 48;
    public static double RECORD_NUMBER = 10;

    private boolean end;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    /** This initializes the drive motors as well as the cache of velocities and the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetry.addData("Press START when ready.","");
        telemetry.addData("pose", follower.getPose());
        follower.update();
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power. */
    @Override
    public void start() {
        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }
        follower.startTeleopDrive(true);
        follower.update();
        end = false;
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run forward enough, these last velocities recorded are
     * averaged and printed.
     */
    @Override
    public void loop() {

        follower.update();

        if (!end) {
            if (Math.abs(follower.getPose().getX()) > DISTANCE) {
                end = true;
                follower.setTeleOpDrive(0, 0, 0, true);
            } else {
                follower.setTeleOpDrive(0,1,0,true);
                double currentVelocity = Math.abs(follower.poseTracker.getLocalizer().getVelocity().getX());
                velocities.add(currentVelocity);
                velocities.remove(0);
            }
        } else {
            double average = 0;
            for (double velocity : velocities) {
                average += velocity;
            }
            average /= velocities.size();
            telemetry.addData("Lateral Velocity: ", average);

        }
    }
}