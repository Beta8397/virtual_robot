package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "QQ_AutoBase")
abstract public class QQ_AutoBase extends OpMode {
    private Robot robot = new Robot();

    private List<QQ_AutoAction> autoSteps;
    private int stepNum;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    abstract List<QQ_AutoAction> getSteps();

    @Override
    public void start() {
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
