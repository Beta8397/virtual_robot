package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc16072.Util.RobotPosition;

import java.util.Arrays;
import java.util.List;
/**
 * Autonomous Parking OpMode contributed by ftc16072.
 * For red alliance: start with robot in square next to the blue depot, facing toward the building zone.
 * For blue alliance: start with robot in square next to the blue building site, facing toward the loading zone.
 */
@Autonomous(name = "Park", group = "ftc16072")
public class AutoParkOpMode extends QQ_AutoBase {
    List<QQ_AutoAction> getSteps() {
        QQ_ActionSetPosition startPosition =
                new QQ_ActionSetPosition(getStartPosition());

        if (farPark) {
            if (startDepot) {
                return Arrays.asList(
                        startPosition,
                        new QQ_ActionDriveTo(farPark_x, START_DEPOT_Y, DistanceUnit.INCH),
                        new QQ_ActionDriveTo(farPark_x, 0, DistanceUnit.INCH));
            } else {
                return Arrays.asList(
                        startPosition,
                        new QQ_ActionDriveTo(farPark_x, START_BUILD_Y, DistanceUnit.INCH),
                        new QQ_ActionDriveTo(farPark_x, 0, DistanceUnit.INCH));
            }
        } else {
            return Arrays.asList(
                    startPosition,
                    new QQ_ActionDriveTo(nearPark_x, 0, DistanceUnit.INCH));
        }
    }
}