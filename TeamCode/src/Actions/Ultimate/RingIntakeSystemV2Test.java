package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
    private static final int[][] STATE_SWITCH = { { ON, REVERSE }, { OFF, REVERSE }, { ON, OFF } };
    
    private int state;

    private MotorController intakeMotor;
    private Servo intakeServo;
    
    private DistanceSensor ringDetector;
    private boolean ringSensed;
    private static final double RING_DETECTION_THRESHOLD = 0;//todo find these
    private static final double NO_RING_THRESHOLD = 0;
    
    public int numRingsTakenIn;

    public RingIntakeSystemV2Test(HardwareMap hardwareMap) {
        try {
            intakeMotor = new MotorController("intake_motor", "MotorConfig/NoLoad40.json", hardwareMap);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            
            intakeServo = hardwareMap.servo.get("intake_servo");
        } catch (IOException e) {
            e.printStackTrace();
        }
        
        state = OFF;
    }
    
    public void dropDown() {
        intakeServo.setPosition(0);
        intakeServo.setPosition(1);
    }
    
    private void updateRobot() {
        intakeMotor.setMotorPower(POWERS[state]);//todo make led lights indicate state
        detectRingsInIntake();
        if (numRingsTakenIn > 3)
            intakeReverse();
    }
    
    //tele-op function
    public void updateState(int buttonPressed) {
        state = STATE_SWITCH[state][buttonPressed];
        updateRobot();
    }
    
    // the following are used in auto
    public void intakeOn() {
        state = ON;
        updateRobot();
    }
    
    public void intakeReverse() {
        state = REVERSE;
        updateRobot();
    }
    
    public void intakeOff() {
        state = OFF;
        updateRobot();
    }
    
    private void detectRingsInIntake() {
        double distanceToRing = ringDetector.getDistance(DistanceUnit.INCH);
        if (ringSensed && distanceToRing > NO_RING_THRESHOLD) {
            ringSensed = false;
        } else if (!ringSensed && distanceToRing < RING_DETECTION_THRESHOLD) {
            ringSensed = true;
            if (state == OFF) {
                numRingsTakenIn--;
            } else if (state == ON) {
                numRingsTakenIn++;
            }
        }
    }

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
