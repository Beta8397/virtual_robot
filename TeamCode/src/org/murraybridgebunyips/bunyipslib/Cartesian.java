package org.murraybridgebunyips.bunyipslib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

/**
 * Rotational utilities for Pose2d and Vector2d.
 *
 * @author Lucas Bubner, 2023
 */
public class Cartesian {
    private Cartesian() {
    }

    public static Pose2d toPose(Pose2d pose) {
        return new Pose2d(pose.getY(), -pose.getX(), -pose.getHeading());
    }

    public static Pose2d fromPose(Pose2d pose) {
        return new Pose2d(-pose.getY(), pose.getX(), -pose.getHeading());
    }

    public static Pose2d toPose(double x, double y, double heading) {
        // noinspection SuspiciousNameCombination
        return new Pose2d(y, -x, -heading);
    }

    public static Pose2d fromPose(double x, double y, double heading) {
        // noinspection SuspiciousNameCombination
        return new Pose2d(-y, x, -heading);
    }

    public static Vector2d toVector(Vector2d vector) {
        return new Vector2d(vector.getY(), -vector.getX());
    }

    public static Vector2d fromVector(Vector2d vector) {
        return new Vector2d(-vector.getY(), vector.getX());
    }

    public static Vector2d toVector(double x, double y) {
        // noinspection SuspiciousNameCombination
        return new Vector2d(y, -x);
    }

    public static Vector2d fromVector(double x, double y) {
        // noinspection SuspiciousNameCombination
        return new Vector2d(-y, x);
    }
}
