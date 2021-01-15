package Actions.Ultimate;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Autonomous.ConfigVariables;
import SensorHandlers.MagneticLimitSwitch;

/**
 * Author: Ethan Fisher
 * Date: 10/21/2020
 *
 * Used for shooting rings
 */
public class ShooterSystemV1 {
	
	public Servo aimServo;
	public static final double HIGHEST_POSITION = 0;
	public static final double POWER_SHOT_POSITION = 0.45;
	public static final double LOWERED_POSITION = 1;
	final double ANGLE_INCREMENT = 0.05;
	
	// good
	public WheelMotor wheelMotor;
	private boolean wheelSpinning;
	private static final int SHOOTER_ON_RPM = 3700;
	private static final int HIGH_GOAL_SPEED = 0;
	private static final int POWER_SHOT_SPEED = 0;
	
	// good
	public CRServo elevatorServo;
	public static final int TOP = 2;
	public static final int MIDDLE = 1;
	public static final int BOTTOM = 0;
	public int elevatorPosition;
	public boolean stayAtTop;
	//public volatile MagneticLimitSwitch elevatorTopSwitch;
	//public volatile MagneticLimitSwitch elevatorBottomSwitch;
	
	// good
	public Servo pinballServo;
	private double pinballAngle;
	public static final double PINBALL_TURNED = 1;
	public static final double PINBALL_REST = 0;
	
	public DistanceSensor ringSensor;
	private static final double RING_DETECTOR_HEIGHT = 5;
	public int numRings;
	
	private volatile boolean shouldRun;
	
	public ShooterSystemV1(HardwareMap hardwareMap, final LinearOpMode mode) {
		aimServo = hardwareMap.servo.get("aimServo");
		wheelMotor = new WheelMotor("shooter_motor", hardwareMap);
		elevatorServo = hardwareMap.crservo.get("elevatorServo");
		//elevatorTopSwitch = new MagneticLimitSwitch(hardwareMap.digitalChannel.get("elevatorTopSwitch"));
		//elevatorBottomSwitch = new MagneticLimitSwitch(hardwareMap.digitalChannel.get("elevatorBottomSwitch"));
		
		pinballServo = hardwareMap.servo.get("shooter_servo");
		
//		ringSensor = hardwareMap.get(DistanceSensor.class, "ring_sensor");
		
		numRings = 0;
		wheelSpinning = false;
		elevatorPosition = BOTTOM;
		pinballAngle = PINBALL_REST;
		stayAtTop = false;
		
		shouldRun = true;
		new Thread(new Runnable() {
			@Override
			public void run() {
				while (shouldRun && mode.opModeIsActive())
					update();
			}
		}).start();
	}
	
	public void toggleWheelPower() {
		wheelSpinning = !wheelSpinning;
		wheelMotor.setRPM(wheelSpinning ? SHOOTER_ON_RPM : 0);
	}
	
	public void turnOnShooterWheel() {
		wheelSpinning = true;
		wheelMotor.setRPM(SHOOTER_ON_RPM);
	}
	
	public void turnOffShooterWheel() {
		wheelSpinning = false;
		wheelMotor.setRPM(0);
	}
	
	public void setPowerShotSpeed() {
		wheelMotor.setRPM(POWER_SHOT_SPEED);
	}
	
	public void setHighGoalSpeed() {
		wheelMotor.setRPM(HIGH_GOAL_SPEED);
	}
	
	// moves the pinball servo
	public void shoot() {
		if (pinballAngle == PINBALL_TURNED)
			pinballAngle = PINBALL_REST;
		else
			pinballAngle = PINBALL_TURNED;
		
		pinballServo.setPosition(pinballAngle);
	}
	
	public void raiseShooter() {
		aimServo.setPosition(aimServo.getPosition() - ANGLE_INCREMENT);
	}
	
	public void lowerShooter() {
		aimServo.setPosition(aimServo.getPosition() + ANGLE_INCREMENT);
	}
	
	public void setShooter(double angle) { aimServo.setPosition(angle); }
	
	public void raiseElevator() {
		if (elevatorPosition != TOP)
			elevatorServo.setPower(-1);
	}
	
	public void lowerElevator() {
		stayAtTop = false;
		if (elevatorPosition != BOTTOM)
			elevatorServo.setPower(1);
	}
	
	public void keepElevatorAtTop() {
		stayAtTop = true;
		raiseElevator();
	}
	
	public void stopElevator() { elevatorServo.setPower(0); }
	
	private void update() {
//        if (elevatorTopSwitch.isActivated() && elevatorPosition != TOP) {
//            elevatorPosition = TOP;
//            elevatorServo.setPower(-0.02);
//        } else if (elevatorBottomSwitch.isActivated() && !stayAtTop) { //watch out for the zero case because then the robot will think its at the bottom when its at the top
//            elevatorPosition = BOTTOM;
//            elevatorServo.setPower(0);
//        } else if (!elevatorTopSwitch.isActivated() && stayAtTop)
//            elevatorServo.setPower(-1);
//
//        if (!elevatorTopSwitch.isActivated() && !elevatorBottomSwitch.isActivated())
//            elevatorPosition = MIDDLE;
		
		wheelMotor.updateShooterRPM();
		
//		numRings = numRingsInHopper();
	}
	
	public double calculateRingVelocity(double xDistance, double yDistance) {
		double temp0 = yDistance - xDistance * Math.tan(ConfigVariables.SHOOTER_ANGLE);
		if (temp0 < 0)
			return 0;
		double temp1 = Math.sqrt(-4.9 * xDistance * temp0);
		double metersPerSecond = Math.cos(ConfigVariables.SHOOTER_ANGLE) / temp1;
		return metersPerSecond * 39.3701;//meters per second * inches per meter = inches per second
	}
	
	public double calculateRPMForExitVelocity(double desiredExitVelocityInchesPerSecond) {
		return desiredExitVelocityInchesPerSecond * 11.0639908;
	}
	
	public double calculateRPM(double xDistance, double yDistance) {
		return calculateRPMForExitVelocity(calculateRingVelocity(xDistance, yDistance));
	}
	
	public void stop() { shouldRun = false; }
	
	private int numRingsInHopper() {
		double stackHeight = RING_DETECTOR_HEIGHT - ringSensor.getDistance(DistanceUnit.INCH);
		return (int)Math.round(stackHeight / .75);
	}
	
	// TODO
//    public void shootWithAdjustedAngle(Location robotLocation) {
//        // targets: top goal, powershots
//
//        double shooterHeightCM = 5;
//        Vector3 robotVector = new Vector3(robotLocation.getX(), robotLocation.getY(), shooterHeightCM);
//
//        Vector3 targetVector;
//        if (aimPosition == POWER_SHOT_POSITION) {
//            targetVector = new Vector3(ConfigVariables.POWER_SHOT_MIDDLE.getX(),
//                    ConfigVariables.POWER_SHOT_MIDDLE.getY(),
//                    ConfigVariables.POWER_SHOT_HEIGHT_CM);
//        } else {
//            targetVector = new Vector3(ConfigVariables.TOP_GOAL.getX(),
//                    ConfigVariables.TOP_GOAL.getY(),
//                    ConfigVariables.TOP_GOAL_HEIGHT_CM);
//        }
//
//        Vector3 differenceVector = robotVector.distanceFromVector(targetVector);
//        double distanceFromTargetCM = differenceVector.length();
//
//        double motorPower = calculateMotorPower(distanceFromTargetCM);
//        double servoAngle = calculateShootingAngle(motorPower, distanceFromTargetCM);
//
//        wheelMotor.setPower(motorPower);
//        aimServo.setPosition(servoAngle);
//    }
//
//    // TODO
//    public double calculateMotorPower(double distanceCM) {
//        // constrain it to like 0.6 to 1 or something
//        return 0;
//    }
//
//    // TODO
//    public double calculateShootingAngle(double motorPower, double distance) {
//        // probably gonna be something like 30 - 40 degrees
//        return 0;
//    }
}
