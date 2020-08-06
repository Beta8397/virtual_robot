package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc16072.Util.RobotPosition;
//import sun.jvm.hotspot.runtime.win32_amd64.Win32AMD64JavaThreadPDAccess;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Waffle", group = "ftc16072")
public class AutoWaffleOpMode extends QQ_AutoBase {
    double WAFFLE_WIDTH = 18.5;
    double FIELD_BOUNDARIES = 72;

    @Override
    public void init_loop() {
        super.init_loop();
        startDepot = false;
    }

    List<QQ_AutoAction> getSteps() {
        List<QQ_AutoAction> steps = new ArrayList<>();
        QQ_ActionSetPosition startPosition =
                new QQ_ActionSetPosition(getStartPosition());
        if (redAlliance) {
            steps.addAll(Arrays.asList(
                    startPosition,
                    new QQ_ActionDriveTo(WAFFLE_RED_X, WAFLLE_RED_Y, DistanceUnit.INCH),
                    new QQ_ActionSnatcher(true),
                    new QQ_ActionDelayFor(0.5),
                    new QQ_ActionDriveTo(WAFFLE_RED_X + 8, WAFLLE_RED_Y, DistanceUnit.INCH),
                    new QQ_ActionRotateTo(90, AngleUnit.DEGREES),
                    new QQ_ActionDriveTo(WAFFLE_RED_X + 8, WAFLLE_RED_Y + 4, DistanceUnit.INCH), //to square on wall
                    new QQ_ActionSetPosition(new RobotPosition(WAFFLE_RED_X + 8, FIELD_BOUNDARIES - WAFFLE_WIDTH, DistanceUnit.INCH, 90, AngleUnit.DEGREES)),
                    new QQ_ActionSnatcher(false),
                    new QQ_ActionDelayFor(0.5)
            ));
        } else {
            steps.addAll(Arrays.asList(
                    startPosition,
                    new QQ_ActionDriveTo(-WAFFLE_RED_X, WAFLLE_RED_Y, DistanceUnit.INCH),
                    new QQ_ActionSnatcher(true),
                    new QQ_ActionDelayFor(0.5),
                    new QQ_ActionDriveTo(-WAFFLE_RED_X - 8, WAFLLE_RED_Y, DistanceUnit.INCH),
                    new QQ_ActionRotateTo(90, AngleUnit.DEGREES),
                    new QQ_ActionDriveTo(-WAFFLE_RED_X - 8, WAFLLE_RED_Y + 4, DistanceUnit.INCH), //to square on wall
                    new QQ_ActionSetPosition(new RobotPosition(-WAFFLE_RED_X - 8, FIELD_BOUNDARIES - WAFFLE_WIDTH, DistanceUnit.INCH, 90, AngleUnit.DEGREES)),
                    new QQ_ActionSnatcher(false),
                    new QQ_ActionDelayFor(0.5)
            ));
        }
        steps.addAll(getParkSteps());
        return steps;
    }

    List<QQ_AutoAction> getParkSteps() {
        if (farPark) {
            return Arrays.asList(
                    new QQ_ActionDriveTo(farPark_x, 0, DistanceUnit.INCH));
        } else {
            return Arrays.asList(
                    new QQ_ActionDriveTo(nearPark_x, FIELD_BOUNDARIES - WAFFLE_WIDTH, DistanceUnit.INCH),
                    new QQ_ActionDriveTo(nearPark_x, 0, DistanceUnit.INCH));
        }
    }
}