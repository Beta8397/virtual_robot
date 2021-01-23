package Actions.Ultimate;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.IOException;

import Actions.ActionHandler;
import MotorControllers.MotorController;

public class RingIntakeSystemV2Test implements ActionHandler {
	
	private static final int MOTOR_POWER = 1;
	
	private static final int OFF = 0;
	private static final int ON = 1;
	private static final int REVERSE = 2;
	
	private static final double[] POWERS = { 0, MOTOR_POWER, -MOTOR_POWER };
	private static final int[][] STATE_SWITCH =   {
			{ ON, REVERSE },
			{ OFF, REVERSE },
			{ ON, OFF }
	};
	private static final RevBlinkinLedDriver.BlinkinPattern[] COLORS = {
			RevBlinkinLedDriver.BlinkinPattern.WHITE,
			RevBlinkinLedDriver.BlinkinPattern.GREEN,
			RevBlinkinLedDriver.BlinkinPattern.RED
	};
	
	private int state;
	
	private MotorController intakeMotor;
	private Servo intakeServo; // use a servo handler here instead
	private RevBlinkinLedDriver driver;

	private RevColorSensorV3 ringDetector;
	private boolean ringSensed;
	private static final double RING_DETECTION_THRESHOLD = 0;//todo find these
	private static final double NO_RING_THRESHOLD = 0;
	
	public int numRingsTakenIn;
	
	public RingIntakeSystemV2Test(HardwareMap hardwareMap) {
		try {
			intakeMotor = new MotorController("intakeMotor", "MotorConfig/NeverRest40.json", hardwareMap);
			intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
			
			intakeServo = hardwareMap.servo.get("intakeServo");
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		state = OFF;
		ringSensed = false;
		
		driver = hardwareMap.get(RevBlinkinLedDriver.class, "intakeLEDs");
	}

	
	public void update() {//call this function repeatedly
		intakeMotor.setMotorPower(POWERS[state]);
		driver.setPattern(COLORS[state]);
		detectRingsInIntake();
	}
	
	private void detectRingsInIntake() {
		double distanceToRing = ringDetector.getDistance(DistanceUnit.INCH);
		if (ringSensed && distanceToRing > NO_RING_THRESHOLD) {
			ringSensed = false;
		} else if (!ringSensed && distanceToRing < RING_DETECTION_THRESHOLD) {
			ringSensed = true;
			if (state == REVERSE) {
				numRingsTakenIn--;
			} else {
				numRingsTakenIn++;
			}
		}
	}
	
	//tele-op function
	public void updateState(int buttonPressed) {//intake on = 0; intake reverse = 1
		state = STATE_SWITCH[state][buttonPressed];
	}
	
	// the following are used in auto
	public void intakeOn() {
		state = ON;
	}
	
	public void intakeReverse() {
		state = REVERSE;
	}
	
	public void intakeOff() {
		state = OFF;
	}

	public void intake() {
		intakeMotor.setMotorPower(1);
	}

	public void spit() {
		intakeMotor.setMotorPower(-1);
	}

	public void pauseIntake() {
		intakeMotor.brake();
	}

	public void intakeServoOut() { intakeServo.setPosition(0); }

	public void intakeServoIn() { intakeServo.setPosition(1); }
	
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
		intakeMotor.killMotorController();
	}
}
