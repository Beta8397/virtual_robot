package org.firstinspires.ftc.teamcode.pedropathing.tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * FOR USE ONLY WITH THE VIRTUAL_ROBOT PROJECT. LOGS LOCALIZER DATA TO THE
 * CONSOLE. THIS WILL NOT WORK IN THE REAL FTC SDK!
 */
public class PedroTuningUtil {
    public static void logLocalizerData(Localizer localizer){
        Pose pose = localizer.pose();
        Velocity vel = localizer.velocity();
        Twist twist = localizer.twist();
        System.out.printf("\nPose: [%.3f %.3f %.3f]  Vel: [%.3f %.3f %.3f]  Twist: [%.3f %.3f %.3f]\n",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()), vel.vx, vel.vy, Math.toRadians(vel.omega),
                twist.vx, twist.vy, Math.toDegrees(vel.omega));
    }

    public static boolean isBusy(Follower follower, boolean turning, Pose target){
        if (!turning || follower.mode() != Follower.Mode.HOLD){
            return follower.isBusy();
        } else {
            Pose p = follower.pose();
            Twist t = follower.twist();
            return Math.hypot(p.x()-target.x(), p.y()-target.y()) > 0.5
                    || Math.abs(AngleUnit.normalizeRadians(p.heading()-target.heading())) > Math.toRadians(1)
                    || Math.hypot(t.vx, t.vy) > 1 || Math.abs(t.omega) > Math.toRadians(1);
        }
    }
}
