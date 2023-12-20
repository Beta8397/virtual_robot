package org.firstinspires.ftc.teamcode.common.roadrunner.util;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;

//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.IOException;
import java.io.InputStream;

/**
 * Set of utilities for loading trajectories from assets (the plugin save location).
 */
public class AssetsTrajectoryManager {

    /**
     * Loads the group config.
     */
    @Nullable
    public static TrajectoryGroupConfig loadGroupConfig() {
//        try {
//           InputStream inputStream = AppUtil.getDefContext().getAssets().open(
//                    "trajectory/" + TrajectoryConfigManager.GROUP_FILENAME);
//            return TrajectoryConfigManager.loadGroupConfig(inputStream);
//        } catch (IOException e) {
//            return null;
//        }
        return null;
    }

    /**
     * Loads a trajectory config with the given name.
     */
    @Nullable
    public static TrajectoryConfig loadConfig(String name) {
//        try {
//            InputStream inputStream = AppUtil.getDefContext().getAssets().open(
//                    "trajectory/" + name + ".yaml");
//            return TrajectoryConfigManager.loadConfig(inputStream);
//        } catch (IOException e) {
//            return null;
//        }
        return null;
    }

    /**
     * Loads a trajectory builder with the given name.
     */
    @Nullable
    public static TrajectoryBuilder loadBuilder(String name) {
        TrajectoryGroupConfig groupConfig = loadGroupConfig();
        TrajectoryConfig config = loadConfig(name);
        if (groupConfig == null || config == null) {
            return null;
        }
        return config.toTrajectoryBuilder(groupConfig);
    }

    /**
     * Loads a trajectory with the given name.
     */
    @Nullable
    public static Trajectory load(String name) {
        TrajectoryBuilder builder = loadBuilder(name);
        if (builder == null) {
            return null;
        }
        return builder.build();
    }
}
