package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc16072.Util.RobotPosition;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Auto Park", group = "ftc16072")
public class AutoParkOpMode extends QQ_AutoBase {

    private boolean startDepot = true;
    private boolean redAliance = true;
    private boolean farPark = true;

    private boolean aPressed;
    private boolean xPressed;
    private boolean yPressed;

    public void init_loop() {
        if (gamepad1.a & !aPressed) {
            startDepot = !startDepot;
        }
        aPressed = gamepad1.a;
        if (gamepad1.x & !xPressed) {
            redAliance = !redAliance;
        }
        xPressed = gamepad1.x;
        if (gamepad1.y & !yPressed) {
            farPark = !farPark;
        }
        yPressed = gamepad1.y;

        telemetry.addData("A = StartDepot, x = Alliance, y = Park", "");
        telemetry.addData("StartDepot, redAliance, farPark", "\n %b, %b, %b", startDepot, redAliance, farPark);

        telemetry.addData("Place robot at:", "%.0f in %.0f in", getStartPosition().getX(DistanceUnit.INCH), getStartPosition().getY(DistanceUnit.INCH));

    }

    private RobotPosition getStartPosition() {
        double startX = 0;
        double startY = 0;

        if (startDepot && redAliance) {
            startX = 60;
            startY = -36;
        }
        if (startDepot && !redAliance) {
            startX = -60;
            startY = -36;
        }
        if (!startDepot && !redAliance) {
            startX = -60;
            startY = 36;
        }
        if (!startDepot && redAliance) {
            startX = 60;
            startY = 36;
        }
        return new RobotPosition(startX, startY, DistanceUnit.INCH, 0, AngleUnit.RADIANS);

    }

    List<QQ_AutoAction> getSteps() {
        QQ_ActionSetPosition startPosition =
                new QQ_ActionSetPosition(getStartPosition().getX(DistanceUnit.INCH),
                        getStartPosition().getY(DistanceUnit.INCH), DistanceUnit.INCH);
        if (redAliance) {
            if (startDepot && farPark) {
                return Arrays.asList(
                        startPosition,
                        new QQ_ActionDriveTo(32, -36, DistanceUnit.INCH),
                        new QQ_ActionDriveTo(32, 0, DistanceUnit.INCH));
            }
            if (startDepot && !farPark) {
                return Arrays.asList(
                        startPosition,
                        new QQ_ActionDriveTo(60, 0, DistanceUnit.INCH));
            }
            if (!startDepot && farPark) {
                return Arrays.asList(
                        startPosition,
                        new QQ_ActionDriveTo(32, 36, DistanceUnit.INCH),
                        new QQ_ActionDriveTo(32, 0, DistanceUnit.INCH));
            }
            if (!startDepot && !farPark) {
                return Arrays.asList(
                        startPosition,
                        new QQ_ActionDriveTo(60, 0, DistanceUnit.INCH));
            }
        } else {
            if (startDepot && farPark) {
                return Arrays.asList(
                        startPosition,
                        new QQ_ActionDriveTo(-32, -36, DistanceUnit.INCH),
                        new QQ_ActionDriveTo(-32, 0, DistanceUnit.INCH));
            }
            if (startDepot && !farPark) {
                return Arrays.asList(
                        startPosition,
                        new QQ_ActionDriveTo(-60, 0, DistanceUnit.INCH));
            }
            if (!startDepot && farPark) {
                return Arrays.asList(
                        startPosition,
                        new QQ_ActionDriveTo(-32, 36, DistanceUnit.INCH),
                        new QQ_ActionDriveTo(-32, 0, DistanceUnit.INCH));
            }
            if (!startDepot && !farPark) {
                return Arrays.asList(
                        startPosition,
                        new QQ_ActionDriveTo(-60, 0, DistanceUnit.INCH));
            }

        }

        return Arrays.asList(
                startPosition
        );
    }

}
