package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RingIntakeSystemV1 {

    // TODO this class should be working rn

    private DcMotor intakeMotor;
    private boolean intakeOn;
    private boolean intakeReversed;
    
    private Servo intakeServo;

    private static final int MOTOR_POWER = 1;
    
    private DistanceSensor ringDetector;
    private boolean ringSensed;
    private static final double RING_DETECTION_THRESHOLD = 0;//todo find these
    private static final double NO_RING_THRESHOLD = 0;
    
    public int numRingsTakenIn;
    
    public RingIntakeSystemV1(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intake_motor");
        intakeOn = false;
        intakeReversed = false;
        
        intakeServo = hardwareMap.servo.get("intake_servo");
    }

    public void toggleIntakePower() {
        // Turn intake motor on or off
        intakeOn = !intakeOn;
        intakeMotor.setPower((intakeOn ? MOTOR_POWER : 0) * (intakeReversed ? -MOTOR_POWER : MOTOR_POWER));
    }

    public void toggleIntake() {
        if(intakeOn) {
            toggleIntakePower();
        }
        else {
            intakeOn = true;
            intakeMotor.setPower(MOTOR_POWER);
        }
    }

    public void toggleOuttake() {
        if(intakeOn) {
            toggleIntakePower();
        }
        else {
            intakeOn = true;
            intakeMotor.setPower(-(MOTOR_POWER));
        }
    }

    // the following are used in auto
    public void forwardIntake() {
        intakeReversed = false;
        intakeMotor.setPower(intakeOn ? MOTOR_POWER : 0);
    }

    public void reverseIntake() {
        intakeReversed = true;
        intakeMotor.setPower(-(intakeOn ? MOTOR_POWER : 0));
    }

    public void turnOn() {
        intakeOn = true;
        intakeMotor.setPower(MOTOR_POWER * (intakeReversed ? -1 : 1));
    }

    public void turnOff() {
        intakeOn = false;
        intakeMotor.setPower(0);
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
            if (intakeReversed && intakeOn) {
                numRingsTakenIn--;
            } else if (intakeOn) {
                numRingsTakenIn++;
            }
        }
    }
}
