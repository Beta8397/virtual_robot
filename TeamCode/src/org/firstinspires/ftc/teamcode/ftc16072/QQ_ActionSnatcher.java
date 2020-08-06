package org.firstinspires.ftc.teamcode.ftc16072;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class QQ_ActionSnatcher extends QQ_AutoAction {
    private Boolean snatch;

    QQ_ActionSnatcher(Boolean snatch) {
        this.snatch = snatch;
    }

    @Override
    boolean run(Robot robot, double gameTime, Telemetry telemetry) {
        if (snatch) {
            telemetry.addData("Snatcher", "lowered");
        } else {
            telemetry.addData("Snatcher", "raised");
        }
        return true;
    }
}
