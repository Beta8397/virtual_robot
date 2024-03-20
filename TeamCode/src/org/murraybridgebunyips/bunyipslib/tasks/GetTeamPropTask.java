package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.tasks.bases.NoTimeoutTask;
import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.TeamProp;

/**
 * Task for detecting on which spike the Team Prop is placed on.
 * This is measured from the starting position, with the camera facing towards the Spike Marks.
 *
 * @author Lucas Bubner, 2023
 * @author Lachlan Paul, 2023
 */
public class GetTeamPropTask extends NoTimeoutTask {
    private final TeamProp teamProp;
    private TeamProp.Positions position;

    /**
     * Constructor for the GetTeamPropTask.
     *
     * @param teamProp The TeamProp vision processor to use.
     */
    public GetTeamPropTask(TeamProp teamProp) {
        this.teamProp = teamProp;
        if (!teamProp.isAttached())
            throw new RuntimeException("Vision processor is not initialised on a vision system");
    }

    public TeamProp.Positions getPosition() {
        return position;
    }

    @Override
    protected void init() {
        // no-op
    }

    @Override
    protected void periodic() {
        if (!teamProp.getData().isEmpty()) {
            // TeamProp will never have more than one data instance
            position = teamProp.getData().get(0).getPosition();
        }
        opMode.addTelemetry("Spike mark reading: %", position);
    }

    @Override
    protected void onFinish() {
        // no-op
    }

    @Override
    protected boolean isTaskFinished() {
        // Init-task timing only
        return false;
    }
}
