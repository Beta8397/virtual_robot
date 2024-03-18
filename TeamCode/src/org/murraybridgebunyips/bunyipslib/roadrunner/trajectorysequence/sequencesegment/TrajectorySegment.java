package org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.Collections;

/**
 * A segment of a trajectory sequence that is a single trajectory.
 */
public final class TrajectorySegment extends SequenceSegment {
    private final Trajectory trajectory;

    /**
     * Create a new trajectory segment
     *
     * @param trajectory The trajectory to use
     */
    public TrajectorySegment(Trajectory trajectory) {
        // Note: Markers are already stored in the `Trajectory` itself.
        // This class should not hold any markers
        super(trajectory.duration(), trajectory.start(), trajectory.end(), Collections.emptyList());
        this.trajectory = trajectory;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }
}
