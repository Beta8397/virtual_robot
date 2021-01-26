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

package UserControlled.Ultimate;

import Misc.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Actions.Ultimate.RingIntakeSystemV2Test;
import Actions.Ultimate.ShooterSystemV2Test;
import Actions.Ultimate.WobbleGrabberV2Test;
import Autonomous.ConfigVariables;
import Autonomous.Location;
import DriveEngine.Ultimate.UltimateNavigation2;
import UserControlled.GamepadController;
import UserControlled.JoystickHandler;

/**
 * Author: Software Team 2020-2021
 *
 * Controls the Ultimate Goal Robot
 *
 * -------------- TLDR ---------------
 * Player One:
 *      joysticks - drive base
 *      start - N/A
 *      a - N/A
 *      b - N/A
 *      x - reset robot x position against left wall
 *      y - reset robot y position against back wall
 *      dpad up/down/left/right - auto power shots
 *      right trigger - shoot
 *      left trigger - slow mode
 *      right bumper - increase rpm by 100
 *      left bumper - decrease rpm by 100
 *
 * Player Two:
 *      joysticks - N/A
 *      a - toggle intake
 *      b - toggle outake
 *      x - toggle shooter
 *      y - toggle wobble grabber
 *      dpad up/down/left/right - wobble grabber positions
 *      right trigger - toggle intake servo
 *      left trigger - N/A
 */

@TeleOp(name="Ultimate V2", group="Competition")
//@Disabled
public class UltimateV2Better extends LinearOpMode {

	// create objects and locally global variables here
	UltimateNavigation2 robot;
	JoystickHandler leftStick, rightStick;
	
	RingIntakeSystemV2Test intake;
	ShooterSystemV2Test shooter;
	WobbleGrabberV2Test grabber;
	
	GamepadController controllerOne, controllerTwo;

	protected static final double MAX_SPEED = 50;
	protected static final double MED_SPEED = 25;
	protected static final double LOW_SPEED = 15;
	protected static final double MIN_SPEED = 5;
	private static final double MIN_MOVEMENT_POWER = .1;
	
	boolean eStop = false, slowMode = false, intakeOn = false, outakeOn = false, y2Pressed = false, x2Pressed = false, toggleShooterWheel = false, toggleWobbleGrabbed = false,
			rt1Pressed = false, rightTriggerPressed = false, toggleIndex = false, toggleIntakeServo = false, rt2Pressed = false, a2Pressed = false, b2Pressed = false,
			dpadD2pressed = false, toggleIncrement = false, dpadU2pressed = false, toggleDecrement = false;
	
	@Override
	public void runOpMode() {
		// initialize objects and variables here
		// also create and initialize function local variables here
		
		// initialize robot
		// TODO get starting angle
		try {
			robot = new UltimateNavigation2(hardwareMap, new Location(0, 0, 0), "RobotConfig/UltimateV2.json");
			Log.d("Robot: ", robot.toString());
		} catch (Exception e) {
			telemetry.addData("Robot Error", e.toString());
			telemetry.update();
		}
		
		// initialize systems
		intake = new RingIntakeSystemV2Test(hardwareMap);
		shooter = new ShooterSystemV2Test(hardwareMap);
		grabber = new WobbleGrabberV2Test(hardwareMap);

		/** ideally we can use these gamepads for inputs, however the logic is flawed within the
		    gamepad class which causes multiple button presses to be necessary for any kind of response */

		// initialize joysticks
		leftStick = new JoystickHandler(gamepad1, JoystickHandler.LEFT_JOYSTICK);
		rightStick = new JoystickHandler(gamepad1, JoystickHandler.RIGHT_JOYSTICK);
		controllerOne = new GamepadController(gamepad1);
		controllerTwo = new GamepadController(gamepad2);
		
		// add any other useful telemetry data or logging data here
		telemetry.addData("Status", "Initialized");
		telemetry.update();
		
		// nothing goes between the above and below lines
		
		waitForStart();

		
		// should only be used for a time keeper or other small things, avoid using this space when possible
		while (opModeIsActive()) {
			// main code goes here
			
			updateEStop();
			if (!eStop) {
				updateEStop();
				controllerOne.update();
				controllerTwo.update();
				
				controlDrive();
				
				updateEStop();
				controllerOne.update();
				controllerTwo.update();
				if (!eStop) {
					playerOneFunctions(controllerOne);
					playerTwoFunctions(controllerTwo);
				}
				telemetry.addData("Wheel Power:", shooter.betterWheelMotorMaybe.getMotorPower());
				telemetry.addData("Robot Heading:", robot.getOrientation());
				telemetry.addData("Wobble Angle:", grabber.arm.getDegree());
				telemetry.update();

				updateEStop();
				if (!eStop)
					updateMiscFunctions();
			}
			
			// telemetry and logging data goes here
			telemetry.update();
		}
		
		// disable/kill/stop objects here
		stopActions();
		intake.kill();
		shooter.kill();
		grabber.kill();
		robot.stopNavigation();
	}
	
	// misc functions here
	private void updateEStop() {
		if ((controllerOne.dpadDownHeld && controllerOne.startHeld) || (controllerTwo.dpadDownHeld && controllerTwo.startHeld))
			eStop = !eStop;
	}
	
	private void controlDrive() {
		if(controllerOne.leftTriggerHeld) slowMode = true;
		else slowMode = false;
		double drivePower = slowMode ? leftStick.magnitude() / 3 : leftStick.magnitude();
		double turnPower = slowMode ? rightStick.x() / 4 : rightStick.x();
		drivePower = (drivePower > MIN_MOVEMENT_POWER) ? drivePower : 0;//slight error in joystick position doesn't move the robot
		turnPower = (Math.abs(turnPower) > MIN_MOVEMENT_POWER) ? turnPower : 0;
		if (!eStop)
			robot.driveOnHeadingWithTurning(leftStick.angle(), drivePower, turnPower);
	}
	
	private void playerOneFunctions(GamepadController controller) {
		if(gamepad1.dpad_up) powerShots();
		else if(gamepad1.dpad_left) {
//			robot.driveToXY(ConfigVariables.POWER_SHOT_MIDDLE_ON_LINE, 25, this);
			powerShotLeft();
		}
		else if(gamepad1.dpad_down) {
//			robot.driveToXY(ConfigVariables.POWER_SHOT_MIDDLE_ON_LINE, 25, this);
			powerShotCenter();
		}
		else if(gamepad1.dpad_right) {
//			robot.driveToXY(ConfigVariables.POWER_SHOT_MIDDLE_ON_LINE, 25, this);
			powerShotRight();
		}


		// Indexer toggle
		if(gamepad1.right_trigger > 0.1 && !rt1Pressed){
			rt1Pressed = true;
			shoot();
		}
		else if(!(gamepad1.right_trigger > 0.1)) {
			rt1Pressed = false;
		}

		// TODO: update this to be actually correct, need to determine which wall to be against and what the x and y values would be
		if(gamepad1.x)
			robot.setLocation(new Location(-23.5, robot.getRobotLocation().getY()));
		
		if(gamepad1.y)
			robot.setLocation(new Location(robot.getRobotLocation().getX(), -70.5));
		
//		if (gamepad1.left_bumper)
//			shooter.wheelMotor.setRPM((int)shooter.wheelMotor.targetRPM - 100);
//
//		if (gamepad1.right_bumper)
//			shooter.wheelMotor.setRPM((int)shooter.wheelMotor.targetRPM + 100);
	}
	
	private void playerTwoFunctions(GamepadController controller) {
		if(gamepad2.a && !a2Pressed) {
			a2Pressed = true;
			intakeOn = !intakeOn;
			outakeOn = false;
		} else if(!gamepad2.a) {
			a2Pressed = false;
		}

		if(gamepad2.b && !b2Pressed) {
			b2Pressed = true;
			outakeOn = !outakeOn;
			intakeOn = false;
		} else if(!gamepad2.b) {
			b2Pressed = false;
		}

		if(intakeOn) intake.intake();
		else if(outakeOn) intake.spit();
		else intake.pauseIntake();

		// Shooter wheel toggle
		if (gamepad2.x && !x2Pressed) {
			x2Pressed = true;
			toggleShooterWheel = !toggleShooterWheel;
		} else if (!gamepad2.x) {
			x2Pressed = false;
		}
		if (toggleShooterWheel) {
			shooter.spinUp();
		} else {
			shooter.pauseShooter();
		}

		// Wobble grab toggle
		if (gamepad2.y && !y2Pressed) {
			y2Pressed = true;
			toggleWobbleGrabbed = !toggleWobbleGrabbed;
		} else if (!gamepad2.y) {
			y2Pressed = false;
		}
		if (toggleWobbleGrabbed) {
			grabber.setClawGrabAngle();
		} else {
			grabber.releaseWobble();
		}

		if (gamepad2.dpad_up)
			grabber.setArmAngle(WobbleGrabberV2Test.WALL_ANGLE);
		else if (gamepad2.dpad_right)
			grabber.setArmAngle(WobbleGrabberV2Test.LIFT_ANGLE);
		else if (gamepad2.dpad_down)
			grabber.setArmAngle(WobbleGrabberV2Test.GRAB_AND_DROP_ANGLE);
		else if (gamepad2.dpad_left)
			grabber.setArmAngle(WobbleGrabberV2Test.INIT_ANGLE);

		// Intake servo toggle
		if (gamepad2.right_trigger > 0.1 && !rt2Pressed) {
			rt2Pressed = true;
			toggleIntakeServo = !toggleIntakeServo;
		} else if (!(gamepad2.right_trigger > 0.1)) {
			rt2Pressed = false;
		}
		if (toggleIntakeServo) {
			intake.intakeServoOut();
		} else {
			intake.intakeServoIn();
		}

		if (gamepad2.right_bumper) {
			shooter.setHighGoalPower();
		}

		if(gamepad2.left_bumper) {
			shooter.setPowerShotPower();
		}
	}
	
	private void shoot() {
		if(shooter.indexServo.getPosition() == 1){
			shooter.setIndexLeft();
		}
		else {
			shooter.setIndexRight();
		}
		intake.numRingsTakenIn--;
	}
	
	private void powerShots() {
		robot.driveToLocationPID(ConfigVariables.POWER_SHOT_LOCATION_NO_HEADING, MED_SPEED, this);
		powerShotLeft();
		powerShotCenter();
		powerShotRight();
	}
	
	// TODO: Modify the functions below to actually go to the correct positions and score power shots
	
	private void powerShotLeft() {
		shooter.setPowerShotPower();
		robot.driveToLocationPID(ConfigVariables.POWER_SHOT_LEFT, MED_SPEED,this);
		shoot();
	}
	
	private void powerShotCenter() {
		shooter.setPowerShotPower();
		robot.driveToLocationPID(ConfigVariables.POWER_SHOT_MIDDLE, MED_SPEED,this);
		shoot();
	}
	
	private void powerShotRight() {
		shooter.setPowerShotPower();
		robot.driveToLocationPID(ConfigVariables.POWER_SHOT_RIGHT, MED_SPEED,this);
		shoot();
	}
	
	private void stopActions() {
		robot.brake();
		intake.intakeOff();
		grabber.pause();
//		shooter.turnOffShooterWheel();
		shooter.pauseShooter();
	}
	
	private void updateMiscFunctions() {
		shooter.update();
		intake.update();
	}
}