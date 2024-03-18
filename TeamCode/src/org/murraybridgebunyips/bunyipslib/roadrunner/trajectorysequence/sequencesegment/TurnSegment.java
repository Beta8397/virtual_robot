package org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.util.Angle;

import java.util.List;

/**
 * A segment of a trajectory sequence that is a single turn.
 */
public final class TurnSegment extends SequenceSegment {
    private final double totalRotation;
    private final MotionProfile motionProfile;

    /**
     * Create a new turn segment
     *
     * @param startPose     The start pose
     * @param totalRotation The total rotation to turn
     * @param motionProfile The motion profile to use
     * @param markers       The markers to use
     */
    public TurnSegment(Pose2d startPose, double totalRotation, MotionProfile motionProfile, List<TrajectoryMarker> markers) {
        super(
                motionProfile.duration(),
                startPose,
                new Pose2d(
                        startPose.getX(), startPose.getY(),
                        Angle.norm(startPose.getHeading() + totalRotation)
                ),
                markers
        );

        this.totalRotation = totalRotation;
        this.motionProfile = motionProfile;
    }

    public double getTotalRotation() {
        return totalRotation;
    }

    public MotionProfile getMotionProfile() {
        return motionProfile;
    }
}
