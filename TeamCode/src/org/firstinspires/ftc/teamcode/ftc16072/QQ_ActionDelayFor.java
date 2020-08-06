package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

class QQ_ActionDelayFor extends QQ_AutoAction {
    double endTime;
    double timeDelay;

    QQ_ActionDelayFor(double timeDelay) {
        this.timeDelay = timeDelay;
    }

    @Override
    boolean run(Robot robot, double gameTime, Telemetry telemetry) {
        if (endTime == 0.0) {
            endTime = gameTime + timeDelay;
        }
        if (gameTime >= endTime) {
            return true;
        }
        return false;
    }
}
