package org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.sequencesegment.SequenceSegment;

import java.util.Collections;
import java.util.List;

/**
 * A sequence of trajectory segments.
 */
public class TrajectorySequence {
    private final List<SequenceSegment> sequenceList;

    /**
     * Create a new trajectory sequence
     *
     * @param sequenceList The list of segments to use
     */
    public TrajectorySequence(List<SequenceSegment> sequenceList) {
        if (sequenceList.isEmpty()) throw new EmptySequenceException();

        this.sequenceList = Collections.unmodifiableList(sequenceList);
    }

    /**
     * Get the start pose of the sequence
     *
     * @return The start pose
     */
    public Pose2d start() {
        return sequenceList.get(0).getStartPose();
    }

    /**
     * Get the end pose of the sequence
     *
     * @return The end pose
     */
    public Pose2d end() {
        return sequenceList.get(sequenceList.size() - 1).getEndPose();
    }

    /**
     * Get the time in seconds for the sequence to execute.
     *
     * @return The duration of the sequence
     */
    public double duration() {
        double total = 0.0;

        for (SequenceSegment segment : sequenceList) {
            total += segment.getDuration();
        }

        return total;
    }

    /**
     * Get the segment at the given index
     *
     * @param i The index
     * @return The segment
     */
    public SequenceSegment get(int i) {
        return sequenceList.get(i);
    }

    /**
     * Get the number of segments in the sequence
     *
     * @return The number of segments
     */
    public int size() {
        return sequenceList.size();
    }
}
