package system.robot.roadrunner_util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.List;

public class HALTrajectory {
    private final Trajectory trajectory;
    private final CoordinateMode coordinateMode;

    public HALTrajectory(Trajectory trajectory, CoordinateMode coordinateMode) {
        this.coordinateMode = coordinateMode;
        this.trajectory = trajectory;
    }

    public Pose2d velocity(double time) {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(trajectory.velocity(time));
    }

    public Pose2d acceleration(double time) {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(trajectory.acceleration(time));
    }

    public double duration() {
        return trajectory.duration();
    }

    public Pose2d start() {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(trajectory.start());
    }

    public Pose2d end() {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(trajectory.end());
    }

    public Pose2d get(double time) {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(trajectory.get(time));
    }

    public List<TrajectoryMarker> getMarkers() {
        return trajectory.getMarkers();
    }

    public Path getPath() {
        return trajectory.getPath(); //TODO HAL Path
    }

    public MotionProfile getProfile() {
        return trajectory.getProfile();
    }

    public Trajectory toRoadrunner() {
        return trajectory;
    }
}
