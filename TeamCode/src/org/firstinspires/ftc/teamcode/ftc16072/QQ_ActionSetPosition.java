package org.firstinspires.ftc.teamcode.ftc16072;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc16072.Util.RobotPosition;


class QQ_ActionSetPosition extends QQ_AutoAction {
    private double x;
    private double y;
    private double supposedAngle;

    QQ_ActionSetPosition(RobotPosition robotPosition) {
        this.x = robotPosition.getX(DistanceUnit.CM);
        this.y = robotPosition.getY(DistanceUnit.CM);
        this.supposedAngle = robotPosition.getHeading(AngleUnit.RADIANS);
    }

    @Override
    boolean run(Robot robot, double gameTime, Telemetry telemetry) {
        robot.nav.setPosition(x, y, DistanceUnit.CM);
        robot.nav.resetIMU(supposedAngle, AngleUnit.RADIANS);
        return true;
    }
}

