package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc16072.Util.RobotPosition;

@TeleOp(name = "mechanum auto opmode", group = "ftc16072")
@Disabled
public class MecanumAutoOpMode extends OpMode {
    private Robot robot = new Robot();
    int state = 0;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
    }


    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        telemetry.addData("X", robot.nav.getEstimatedPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Y", robot.nav.getEstimatedPosition().getY(DistanceUnit.INCH));
        telemetry.addData("state", state);
        switch (state) {
            case 0:
                if (robot.nav.driveTo(24, 0, DistanceUnit.INCH)) {
                    state = 1;
                }
                break;
            case 1:
                if (robot.nav.rotateTo(90, AngleUnit.DEGREES)) {
                    state = 2;
                }
                break;
            case 2:
                if (robot.nav.driveTo(0, 0, DistanceUnit.INCH)) {
                    state = 3;
                }
                break;
            case 3:

                if (robot.nav.driveTo(-24, -24, DistanceUnit.INCH)) {
                    state = 4;
                }
                break;

        }
    }
}
