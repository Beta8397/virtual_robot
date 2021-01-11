package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RingIntakeSystemV1 {

    // TODO this class should be working rn

    private DcMotor intakeMotor;
    private static final int OFF = 0;
    private static final int ON = 1;
    private static final int REVERSE = 2;
    private static final int[][] STATE_SWITCH =   {
            { ON, REVERSE },
            { OFF, REVERSE },
            { ON, OFF }
    };
    
    private static final int MOTOR_POWER = 1;
    private static final double[] POWERS = { 0, MOTOR_POWER, -MOTOR_POWER };
    
    private int state;
    
    private Servo intakeServo;

    
    private DistanceSensor ringDetector;
    private boolean ringSensed;
    private static final double RING_DETECTION_THRESHOLD = 0;//todo find these
    private static final double NO_RING_THRESHOLD = 0;
    
    public int numRingsTakenIn;
    
    public RingIntakeSystemV1(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intake_motor");
        
        state = OFF;
        
        intakeServo = hardwareMap.servo.get("intake_servo");
        
        ringDetector = hardwareMap.get(DistanceSensor.class, "intakeSensor");
    }
    
    public void update() {//call this function repeatedly
        intakeMotor.setPower(POWERS[state]);
        detectRingsInIntake();
        if (numRingsTakenIn > 3)
            intakeReverse();
    }

    public void toggleIntake() {
        updateState(0);
    }

    public void toggleOuttake() {
        updateState(1);
    }

    public void turnOn() {
        intakeOn();
    }

    public void turnOff() {
        intakeOff();
    }
    
    public void dropDown() {
        intakeServo.setPosition(0);
        intakeServo.setPosition(1);
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
}
