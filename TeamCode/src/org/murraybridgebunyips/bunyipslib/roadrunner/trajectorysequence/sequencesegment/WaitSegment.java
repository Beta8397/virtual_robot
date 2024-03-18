package org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.List;

/**
 * A segment of a trajectory sequence that is a single wait.
 */
public final class WaitSegment extends SequenceSegment {
    /**
     * @param pose    The pose
     * @param seconds The number of seconds to wait
     * @param markers The markers to use
     */
    public WaitSegment(Pose2d pose, double seconds, List<TrajectoryMarker> markers) {
        super(seconds, pose, pose, markers);
    }
}
