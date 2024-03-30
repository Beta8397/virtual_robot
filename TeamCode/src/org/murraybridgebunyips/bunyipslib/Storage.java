package org.murraybridgebunyips.bunyipslib;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

import java.util.ArrayList;
import java.util.List;

/**
 * Global storage utilities for robot operation.
 * Runtime options here may be modified or viewed via FtcDashboard.
 *
 * @author Lucas Bubner, 2024
 */
@Config
public final class Storage {
    /**
     * Static array of hardware errors stored via hardware name.
     *
     * @see RobotConfig
     */
    public static final ArrayList<String> hardwareErrors = new ArrayList<>();

    /**
     * Components that are unusable and should not have their errors logged.
     *
     * @see NullSafety
     */
    public static final List<String> unusableComponents = new ArrayList<>();

    /**
     * The last known position of the robot from odometry.
     *
     * @see RoadRunnerDrive
     */
    public static Pose2d lastKnownPosition = null;

    // TODO: Persistent storage options for robot data

    private Storage() {
    }
}
