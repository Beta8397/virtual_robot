package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.IOException;

import Actions.ActionHandler;
import Actions.HardwareWrappers.ServoHandler;
import Autonomous.ConfigVariables;
import MotorControllers.MotorController;

/**
 * Author: Jordan Martin
 * Date: 12/19/2020
 *
 * Used for shooting rings
 */
public class ShooterSystemV2Test implements ActionHandler {
	// good
	public WheelMotor wheelMotor;
	private boolean wheelSpinning;
	public MotorController betterWheelMotorMaybe;
	
	// TODO fix PID controller in rpm class
	private static final int SHOOTER_ON_SPEED = 5000; // rotations per minute
	private static final int SHOOTER_OFF_SPEED = 0;
	private static final int HIGH_GOAL_SPEED = 0;
	private static final int POWER_SHOT_SPEED = 0;


	// TODO USE THESE POWERS
	private static final double SHOOTER_OFF_POWER = 0;
	public static final double HIGH_GOAL_POWER = 0.66;
	public static final double POWER_SHOT_POWER = 0.6;
	private double power = HIGH_GOAL_POWER;
	
	// good
	public ServoHandler indexServo;
	private double indexAngle;
	public static final double INDEX_LEFT = -1;
	public static final double INDEX_RIGHT = 1;


//	private RevBlinkinLedDriver ringCount;
//	private DistanceSensor ringDetector;
//	private static final double RING_DETECTOR_HEIGHT = 0;//todo find this value
//	private static final RevBlinkinLedDriver.BlinkinPattern[] COLORS = {
//			RevBlinkinLedDriver.BlinkinPattern.BLACK,
//			RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY,
//			RevBlinkinLedDriver.BlinkinPattern.GRAY,
//			RevBlinkinLedDriver.BlinkinPattern.WHITE
//	};
	
	public ShooterSystemV2Test(HardwareMap hardwareMap) {
//		wheelMotor = new WheelMotor("wheelMotor", hardwareMap);
		try {
			betterWheelMotorMaybe = new MotorController("wheelMotor", "MotorConfig/NeverRest40.json", hardwareMap);
			betterWheelMotorMaybe.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			betterWheelMotorMaybe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
			betterWheelMotorMaybe.setDirection(DcMotorSimple.Direction.FORWARD);
		} catch (IOException e) {
			e.printStackTrace();
		}

		indexServo = new ServoHandler("indexServo", hardwareMap);
		indexServo.setDirection(Servo.Direction.FORWARD);
		
		wheelSpinning = false;
		indexAngle = INDEX_LEFT;
		
//		ringDetector = hardwareMap.get(DistanceSensor.class, "ringDetector");
	}

	public void spinUp() {
		betterWheelMotorMaybe.setMotorPower(power);
	}

	public void incrementPower() {
		if(power <= 0.9) {
			power = power + 0.1;
		}
	}

	public void decrementPower() {
		if(power >= 0.1){
			power = power - 0.1;
		}
	}

	public void pauseShooter() {
		betterWheelMotorMaybe.brake();
	}

	public void toggleShooterWheel() {
		if (wheelMotor.targetRPM == 0)
			wheelMotor.setRPM(SHOOTER_ON_SPEED);
		else
			wheelMotor.setRPM(SHOOTER_OFF_SPEED);
	}

	// moves the index servo
	public void setIndexLeft(){
		indexServo.setPosition(INDEX_LEFT);
		indexAngle = INDEX_LEFT;
	}

	public void setIndexRight(){
		indexServo.setPosition(INDEX_RIGHT);
		indexAngle = INDEX_RIGHT;
	}
	
	public void setPowerShotPower() {
		power = POWER_SHOT_POWER;
	}

	public void setHighGoalPower() {
		power = HIGH_GOAL_POWER;
	}

	public void update() {
		wheelMotor.updateShooterRPM();

//		updateRingLEDs();
	}
	
	public double calculateRingVelocity(double xDistance, double yDistance) {
		double temp0 = yDistance - xDistance * Math.tan(ConfigVariables.SHOOTER_ANGLE);
		if (temp0 < 0)
			return 0;
		double temp1 = Math.sqrt(-4.9 * xDistance * temp0);
		return Math.cos(ConfigVariables.SHOOTER_ANGLE) / temp1;
	}
	
//	private void updateRingLEDs() {
//		double stackHeight = RING_DETECTOR_HEIGHT - ringDetector.getDistance(DistanceUnit.INCH);
//		int numRings = (int)Math.round(stackHeight / .75);
//		ringCount.setPattern(COLORS[numRings]);
//	}
	
	@Override
	public boolean doAction(String action, long maxTimeAllowed) {
		return false;
	}
	
	@Override
	public boolean stopAction(String action) {
		return false;
	}
	
	@Override
	public boolean startDoingAction(String action) {
		return false;
	}
	
	@Override
	public void kill() {
		betterWheelMotorMaybe.killMotorController();
	}
}
