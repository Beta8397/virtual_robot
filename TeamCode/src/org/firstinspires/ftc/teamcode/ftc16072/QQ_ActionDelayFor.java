package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

class QQ_ActionDelayFor extends QQ_AutoAction {
    private double timeDelay;
    private double startTime;

    QQ_ActionDelayFor(double timeDelay) {
        this.timeDelay = timeDelay;
        this.startTime = 0.0;
    }

    @Override
    boolean run(Robot robot, double gameTime, Telemetry telemetry) {
        if (startTime == 0.0) {
            startTime = gameTime;
        }
        telemetry.addData("Time elapsed: ", gameTime - startTime);
        return ((startTime + timeDelay) < gameTime);
    }
}
