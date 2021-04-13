package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import MotorControllers.MotorController;

/**
 * Author: Ethan Fisher
 * Date: 10/21/2020
 *
 * Used for grabbing and releasing the wobble goal
 */
public class WobbleGrabberV1 extends Thread {

    public Servo claw;
    public MotorController arm;
//    private RevColorSensorV3 colorSensor;

    private static final double ARM_POWER_DOWN = -.2;
    private static final double ARM_POWER_UP = .25;

    public static final double CLAW_GRAB_ANGLE = 0.0;
    public static final double CLAW_RELEASE_ANGLE = .9;
    public static final double ANGLE_INCREMENT = 25;
    public static final double VERTICAL = 50;
    public static final double LOWERED_ANGLE = 155;
    public static final double RAISED_ANGLE = 0;
    public static final double LIFTED_ANGLE = 130;

    public boolean wobbleGrabbed;
    private boolean armIsUp;

    public WobbleGrabberV1(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("hand_servo");

        // don't worry about this line. Just test what's currently in the file

        try {
            arm = new MotorController("arm_motor", "ActionConfig/WobbleArmConfig.json", hardwareMap);
        } catch (Exception e) {
            e.printStackTrace();
        }
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        colorSensor = hardwareMap.get(RevColorSensorV3.class, "redSensor");

        wobbleGrabbed = false;
        armIsUp = true;
    }

    public void lowerArm() {
        arm.setPositionDegrees(LOWERED_ANGLE, ARM_POWER_DOWN); // Brooks said motor speed was too fast, thus the 3/4 power but can be adjusted later
    }

    public void liftArm() {
        arm.setPositionDegrees(LIFTED_ANGLE, ARM_POWER_UP);
    }

    public void changeArmState() {
        if (armIsUp)
            lowerArm();
        else {
            raiseArm();
        }
        armIsUp = !armIsUp;
    }

    public void raiseArm() {
        arm.setPositionDegrees(RAISED_ANGLE, ARM_POWER_UP); // Brooks said motor speed was too fast, thus the 3/4 power but can be adjusted later
    }

    public void raiseToVertical() {
        arm.setPositionDegrees(VERTICAL, ARM_POWER_UP);
    }

    public void decreaseAngle() {
        arm.setPositionDegrees(arm.getDegree() + ANGLE_INCREMENT, ARM_POWER_DOWN);
    }

    public void increaseAngle() {
        arm.setPositionDegrees(arm.getDegree() - ANGLE_INCREMENT, ARM_POWER_UP);
    }

    public void setArmAngle(double angle) {
        if (arm.getDegree() - angle > 0)
            arm.setPositionDegrees(angle, ARM_POWER_UP);
        else
            arm.setPositionDegrees(angle, ARM_POWER_DOWN);
    }

    public void grabWobbleGoal() { claw.setPosition(CLAW_GRAB_ANGLE); }
    public void releaseWobbleGoal() { claw.setPosition(CLAW_RELEASE_ANGLE); }

    public void grabOrReleaseWobbleGoal() {
        if (wobbleGrabbed)
            releaseWobbleGoal();
        else
            grabWobbleGoal();
        wobbleGrabbed = !wobbleGrabbed;
    }

    public boolean armIsBusy() {
        return arm.isBusy();
    }

//    public boolean shouldGrabWobbleGoal() {
//        return colorSensor.red() > 200 && colorSensor.blue() < 100 && colorSensor.green() < 100;
//    }

}