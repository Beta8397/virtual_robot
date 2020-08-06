package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc16072.Util.RobotPosition;

import java.util.List;

abstract public class QQ_AutoBase extends OpMode {
    private Robot robot = new Robot();

    private List<QQ_AutoAction> autoSteps;
    private int stepNum;

    protected boolean startDepot = true;
    protected boolean redAlliance = true;
    protected boolean farPark = true;
    protected double farPark_x;
    protected double nearPark_x;


    private boolean aPressed;
    private boolean xPressed;
    private boolean yPressed;

    static double START_DEPOT_Y = -36;
    static double START_BUILD_Y = 36;
    static double BLUE_START_X = -63;
    static double FAR_PARK_RED_X = 36;
    static double NEAR_PARK_RED_X = 62;
    static double WAFFLE_RED_X = 33;
    static double WAFLLE_RED_Y = 48;

    public void init_loop() {
        if (gamepad1.a & !aPressed) {
            startDepot = !startDepot;
        }
        aPressed = gamepad1.a;
        if (gamepad1.x & !xPressed) {
            redAlliance = !redAlliance;
        }
        xPressed = gamepad1.x;
        if (gamepad1.y & !yPressed) {
            farPark = !farPark;
        }
        yPressed = gamepad1.y;

        telemetry.addData("A = StartDepot, x = Alliance, y = Park", "");
        telemetry.addData("Settings", "\n%s, %s, %s",
                startDepot ? "Depot" : "Build",
                redAlliance ? "Red" : "Blue",
                farPark ? "Far" : "Near");

        telemetry.addData("Place robot at:", "%.0f in %.0f in", getStartPosition().getX(DistanceUnit.INCH), getStartPosition().getY(DistanceUnit.INCH));
    }

    protected RobotPosition getStartPosition() {
        double startX = BLUE_START_X;
        double startY = 0;
        double heading = 0;

        if (redAlliance) {
            startX = -1 * startX;
            heading = 180;
        }

        if (startDepot) {
            startY = START_DEPOT_Y;
        } else {
            startY = START_BUILD_Y;
        }
        return new RobotPosition(startX, startY, DistanceUnit.INCH, heading, AngleUnit.DEGREES);
    }

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    abstract List<QQ_AutoAction> getSteps();

    @Override
    public void start() {
        if (redAlliance) {
            farPark_x = FAR_PARK_RED_X;
            nearPark_x = NEAR_PARK_RED_X;
        } else {
            farPark_x = FAR_PARK_RED_X * -1;
            nearPark_x = NEAR_PARK_RED_X * -1;
        }
        autoSteps = getSteps();
        stepNum = 0;

    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        if (stepNum < autoSteps.size()) {
            QQ_AutoAction step = autoSteps.get(stepNum);
            telemetry.addData("auto", stepNum);
            if (step.run(robot, time, telemetry)) {
                stepNum++;
            }
        } else {
            telemetry.addData("auto", "Finished");
        }
    }
}
