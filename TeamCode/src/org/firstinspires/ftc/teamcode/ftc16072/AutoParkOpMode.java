package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Autonomous Parking OpMode contributed by ftc16072.
 * For red alliance: start with robot in square next to the blue depot, facing toward the building zone.
 * For blue alliance: start with robot in square next to the blue building site, facing toward the loading zone.
 */

@TeleOp(name = "auto park opmode", group = "ftc16072")
public class AutoParkOpMode extends OpMode {
    private Robot robot = new Robot();
    int state = 0;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void start() {
        robot.nav.setPosition(60, -36, DistanceUnit.INCH);
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        telemetry.addData("X", robot.nav.getEstimatedPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Y", robot.nav.getEstimatedPosition().getY(DistanceUnit.INCH));
        telemetry.addData("state", state);
        switch (state) {
            case 0:
                if (robot.nav.driveTo(32, -36, DistanceUnit.INCH)) {
                    state = 1;
                }
                break;
            case 1:
                if (robot.nav.driveTo(32, -0, DistanceUnit.INCH)) {
                    state = 2;
                }
                break;
        }
    }
}
