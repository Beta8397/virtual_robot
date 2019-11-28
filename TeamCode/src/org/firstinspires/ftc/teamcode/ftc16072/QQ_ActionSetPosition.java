package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


class QQ_ActionSetPosition extends QQ_AutoAction {
    private double x;
    private double y;
    private DistanceUnit distanceUnit;

    QQ_ActionSetPosition(double x, double y, DistanceUnit distanceUnit) {
        this.x = x;
        this.y = y;
        this.distanceUnit = distanceUnit;
    }

    @Override
    boolean run(Robot robot, double gameTime, Telemetry telemetry) {
        robot.nav.setPosition(x, y, distanceUnit);
        return true;
    }
}

