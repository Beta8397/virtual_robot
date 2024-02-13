package org.murraybridgebunyips.bunyipslib;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Control a set of two servo claws together.
 */
public class DualClaws extends BunyipsSubsystem {
    private final Servo left;
    private final Servo right;
    private final double LEFT_SERVO_CLOSED_POSITION;
    private final double LEFT_SERVO_OPEN_POSITION;
    private final double RIGHT_SERVO_CLOSED_POSITION;
    private final double RIGHT_SERVO_OPEN_POSITION;
    private double leftServoPosition;
    private double rightServoPosition;

    public DualClaws(Servo left, Servo right, double leftClosed, double leftOpen, double rightClosed, double rightOpen) {
        this.left = left;
        this.right = right;
        LEFT_SERVO_CLOSED_POSITION = leftClosed;
        LEFT_SERVO_OPEN_POSITION = leftOpen;
        RIGHT_SERVO_CLOSED_POSITION = rightClosed;
        RIGHT_SERVO_OPEN_POSITION = rightOpen;

        // Always close on init
        leftServoPosition = leftClosed;
        rightServoPosition = rightClosed;
        update();
    }

    public void toggleServo(ServoSide servo) {
        if (servo == ServoSide.LEFT) {
            leftServoPosition = (leftServoPosition == LEFT_SERVO_OPEN_POSITION) ? LEFT_SERVO_CLOSED_POSITION : LEFT_SERVO_OPEN_POSITION;
            return;
        }
        if (servo == ServoSide.RIGHT) {
            rightServoPosition = (rightServoPosition == RIGHT_SERVO_OPEN_POSITION) ? RIGHT_SERVO_CLOSED_POSITION : RIGHT_SERVO_OPEN_POSITION;
            return;
        }
        leftServoPosition = (leftServoPosition == LEFT_SERVO_OPEN_POSITION) ? LEFT_SERVO_CLOSED_POSITION : LEFT_SERVO_OPEN_POSITION;
        rightServoPosition = (rightServoPosition == RIGHT_SERVO_OPEN_POSITION) ? RIGHT_SERVO_CLOSED_POSITION : RIGHT_SERVO_OPEN_POSITION;
    }

    public void openServo(ServoSide servo) {
        if (servo == ServoSide.LEFT) {
            leftServoPosition = LEFT_SERVO_OPEN_POSITION;
            return;
        }
        if (servo == ServoSide.RIGHT) {
            rightServoPosition = RIGHT_SERVO_OPEN_POSITION;
            return;
        }
        leftServoPosition = LEFT_SERVO_OPEN_POSITION;
        rightServoPosition = RIGHT_SERVO_OPEN_POSITION;
    }

    public void closeServo(ServoSide servo) {
        if (servo == ServoSide.LEFT) {
            leftServoPosition = LEFT_SERVO_CLOSED_POSITION;
            return;
        }
        if (servo == ServoSide.RIGHT) {
            rightServoPosition = RIGHT_SERVO_CLOSED_POSITION;
            return;
        }
        leftServoPosition = LEFT_SERVO_CLOSED_POSITION;
        rightServoPosition = RIGHT_SERVO_CLOSED_POSITION;
    }

    /**
     * Push stateful changes to the servos.
     */
    public void update() {
        left.setPosition(leftServoPosition);
        right.setPosition(rightServoPosition);
        opMode.addTelemetry("Servos: L_% R_%", left.getPosition() == LEFT_SERVO_OPEN_POSITION ? "OPEN" : "CLOSE", right.getPosition() == RIGHT_SERVO_OPEN_POSITION ? "OPEN" : "CLOSE");
    }

    public enum ServoSide {
        LEFT,
        RIGHT,
        BOTH
    }
}
