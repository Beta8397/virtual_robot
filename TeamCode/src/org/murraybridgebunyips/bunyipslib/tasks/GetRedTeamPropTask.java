package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;
import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.RedTeamProp;

public class GetRedTeamPropTask extends ForeverTask {
    private final RedTeamProp redTeamProp;
    private volatile TeamPropPositions position = TeamPropPositions.LEFT_SPIKE;

    public TeamPropPositions getPosition() {
        return position;
    }

    public GetRedTeamPropTask(RedTeamProp redTeamProp) {
        this.redTeamProp = redTeamProp;
    }

    public enum TeamPropPositions {
        LEFT_SPIKE,
        CENTER_SPIKE,
        RIGHT_SPIKE
    }

    @Override
    protected void init() {
        if (!redTeamProp.isAttached()) {
            throw new IllegalStateException("RedTeamProp not attached to an active vision processor");
        }
    }

    @Override
    protected void periodic() {
        ContourData biggestContour = ContourData.getLargest(redTeamProp.getData());
        if (biggestContour != null) {
            Dbg.log(biggestContour.getYaw());
            position = biggestContour.getYaw() > 0.5 ? TeamPropPositions.RIGHT_SPIKE : TeamPropPositions.CENTER_SPIKE;
        }
    }

    @Override
    protected void onFinish() {
        // no-op
    }
}
