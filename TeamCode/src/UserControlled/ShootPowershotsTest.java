/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package UserControlled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Actions.Ultimate.RingIntakeSystemV1;
import Actions.Ultimate.ShooterSystemV1;
import Actions.Ultimate.WobbleGrabberV1;
import Autonomous.ConfigVariables;
import Autonomous.Location;
import DriveEngine.Ultimate.UltimateNavigation;

@TeleOp(name="Shoot powershots test", group="Linear Opmode")
//@Disabled
public class ShootPowershotsTest extends LinearOpMode {
    // create objects and locally global variables here

    GamepadController controller;
    UltimateNavigation robot;

    RingIntakeSystemV1 intake;
    ShooterSystemV1 shooter;
    WobbleGrabberV1 grabber;

    Action logHeadingAction, logMotorAction;

    // TODO change this if you want to move in front of the powershots
    //  before shooting, or shoot from where you currently are
    boolean useCurLocation = false;

    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here

        controller = new GamepadController(gamepad1);
        robot = new UltimateNavigation(hardwareMap, new Location(0, 0, UltimateNavigation.NORTH), "RobotConfig/UltimateV1.json");

        // initialize systems
        intake = new RingIntakeSystemV1(hardwareMap);
        shooter = new ShooterSystemV1(hardwareMap, this);
        grabber = new WobbleGrabberV1(hardwareMap);

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // nothing goes between the above and below lines

        waitForStart();

        // add functions to the controller
        controller.OnLeftStickMove = controller.OnRightStickMove = this::controlDrive;
        controller.OnAPressed = () -> shootAllPowershots(useCurLocation);
        controller.OnBPressed = () -> shootPowershot(0, useCurLocation);
        controller.OnXPressed = () -> shootPowershot(1, useCurLocation);
        controller.OnYPressed = () -> shootPowershot(2, useCurLocation);
        controller.OnRightBumperPressed = () -> useCurLocation = !useCurLocation;

        // add update functions to robot
        logHeadingAction = robot::logHeading;
        logMotorAction = robot::logMotorPowers;
        robot.addUpdateAction(logHeadingAction);
        robot.addUpdateAction(logMotorAction);

        while (opModeIsActive()) controller.update();

        robot.stopNavigation();
    }

    private void controlDrive(double x, double y) {
        double drivePower = controller.calculateLeftStickMagnitude();
        double turnPower = gamepad1.right_stick_x;
        robot.driveOnHeadingWithTurning(controller.calculateLeftStickAngle(), -drivePower, -turnPower);
    }

    private void shootAllPowershots(boolean fromCurLocation) {
        shootPowershot(0, fromCurLocation);
        shootPowershot(1, fromCurLocation);
        shootPowershot(2, fromCurLocation);
    }

    private void shootPowershot(int index, boolean fromCurLocation) {

        shooter.keepElevatorAtTop();

        Location powerShotLocation;
        Location toDrive;
        if (index == 0) {
            toDrive = ConfigVariables.POWER_SHOT_LEFT_ON_LINE;
            powerShotLocation = ConfigVariables.POWER_SHOT_LEFT;
        }
        else if (index == 1) {
            toDrive = ConfigVariables.POWER_SHOT_MIDDLE_ON_LINE;
            powerShotLocation = ConfigVariables.POWER_SHOT_MIDDLE;
        }
        else {
            toDrive = ConfigVariables.POWER_SHOT_RIGHT_ON_LINE;
            powerShotLocation = ConfigVariables.POWER_SHOT_RIGHT;
        }

        if (!fromCurLocation)
            robot.driveToLocation(toDrive, UltimateNavigation.MAX_SPEED, this);

        robot.removeUpdateAction(logMotorAction);
        robot.turnToLocation(powerShotLocation, this);
        robot.addUpdateAction(logMotorAction);

        // check if the shooter motor is already on or not
        if (shooter.wheelMotor.motor.getPower() == 0) {
            shooter.turnOnShooterWheel();
            sleep(1000);
        }

        shooter.shoot();
        sleep(500);
        shooter.shoot();
    }
}
