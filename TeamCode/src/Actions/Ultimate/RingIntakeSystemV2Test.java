package Actions.Ultimate;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.IOException;

import javax.print.attribute.standard.JobOriginatingUserName;

import Actions.ActionHandler;
import MotorControllers.MotorController;

public class RingIntakeSystemV2Test implements ActionHandler {
	
	private static final int MOTOR_POWER = 1;
	
	private static final int OFF = 0;
	private static final int ON = 1;
	private static final int REVERSE = 2;
	
	public static final int ON_BUTTON = 0;
	public static final int OFF_BUTTON = 1;
	
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
	
	private MotorController intakeMotor, rollerMotor;
	private Servo intakeServo; // use a servo handler here instead
	private RevBlinkinLedDriver driver;

	public DistanceSensor ringDetector;
	private boolean ringSensed;
	private boolean ringDetected;
	private static final double RING_DETECTION_THRESHOLD = 140;//todo find these
	private static final double NO_RING_THRESHOLD = 160;
	
	public int numRingsTakenIn;
	
	public RingIntakeSystemV2Test(HardwareMap hardwareMap) {
		try {
			intakeMotor = new MotorController("intakeMotor", "MotorConfig/NeverRest40.json", hardwareMap);
			intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
			
			rollerMotor = new MotorController("rollerMotor", "MotorConfig/NeverRest40.json", hardwareMap);
			rollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // there is no encoder on this motor currently
			rollerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			rollerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
			
			intakeServo = hardwareMap.servo.get("intakeServo");
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		state = OFF;
		ringSensed = false;
		
		numRingsTakenIn = 3;
		
		driver = hardwareMap.get(RevBlinkinLedDriver.class, "intakeLEDs");
		
		ringDetector = hardwareMap.get(DistanceSensor.class, "intakeSensor");
	}

	
	public void update() {//call this function repeatedly
		intakeMotor.setMotorPower(POWERS[state]);
		rollerMotor.setMotorPower(POWERS[state]);
		driver.setPattern(COLORS[state]);
		detectRingsInIntake();
		outtakeExtraRing();
	}
	
	private void outtakeExtraRing() {
		if (numRingsTakenIn > 3) {
			intakeReverse();
			numRingsTakenIn--;
		}
	}
	
	private void detectRingsInIntake() {
		double distance = ringDetector.getDistance(DistanceUnit.MM);
		ringDetected = (distance < RING_DETECTION_THRESHOLD) || ringDetected;//detect if ring is present
		ringDetected = !(distance > NO_RING_THRESHOLD) && ringDetected;//detect if ring is absent
		
		if (ringDetected ^ ringSensed) {//edge detection
			if (ringDetected) {//test if the edge is rising or falling
				ringSensed = true;
				numRingsTakenIn++;
			} else {
				ringSensed = false;
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

	public void pauseIntake() {
		intakeMotor.brake();
		rollerMotor.brake();
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
		rollerMotor.killMotorController();
	}
}