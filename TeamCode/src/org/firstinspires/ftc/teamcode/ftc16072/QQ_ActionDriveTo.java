package org.firstinspires.ftc.teamcode.ftc16072;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

class QQ_ActionDriveTo extends QQ_AutoAction {
    private double x;
    private double y;
    DistanceUnit distanceUnit;

    QQ_ActionDriveTo(double x, double y, DistanceUnit distanceUnit) {
        this.x = x;
        this.y = y;
        this.distanceUnit = distanceUnit;
    }

    @Override
    boolean run(Robot robot, double gameTime, Telemetry telemetry) {
        return robot.nav.driveTo(x, y, distanceUnit);
    }
}
