package org.murraybridgebunyips.bunyipslib;

import com.qualcomm.robotcore.hardware.Servo;

import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * Control a set of two servos together.
 */
public class DualServos extends BunyipsSubsystem {
    private Servo left;
    private Servo right;
    private double LEFT_SERVO_CLOSED_POSITION;
    private double LEFT_SERVO_OPEN_POSITION;
    private double RIGHT_SERVO_CLOSED_POSITION;
    private double RIGHT_SERVO_OPEN_POSITION;
    private double leftServoPosition;
    private double rightServoPosition;

    /**
     * Create a new DualServos.
     *
     * @param left        the left servo
     * @param right       the right servo
     * @param leftClosed  the left servo closed position
     * @param leftOpen    the left servo open position
     * @param rightClosed the right servo closed position
     * @param rightOpen   the right servo open position
     */
    public DualServos(Servo left, Servo right, double leftClosed, double leftOpen, double rightClosed, double rightOpen) {
        if (!assertParamsNotNull(left, right)) return;
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

    /**
     * Toggle the state of the servos.
     *
     * @param servo the servo to toggle
     * @return this
     */
    public DualServos toggleServo(ServoSide servo) {
        if (servo == ServoSide.LEFT) {
            leftServoPosition = (leftServoPosition == LEFT_SERVO_OPEN_POSITION) ? LEFT_SERVO_CLOSED_POSITION : LEFT_SERVO_OPEN_POSITION;
            return this;
        }
        if (servo == ServoSide.RIGHT) {
            rightServoPosition = (rightServoPosition == RIGHT_SERVO_OPEN_POSITION) ? RIGHT_SERVO_CLOSED_POSITION : RIGHT_SERVO_OPEN_POSITION;
            return this;
        }
        leftServoPosition = (leftServoPosition == LEFT_SERVO_OPEN_POSITION) ? LEFT_SERVO_CLOSED_POSITION : LEFT_SERVO_OPEN_POSITION;
        rightServoPosition = (rightServoPosition == RIGHT_SERVO_OPEN_POSITION) ? RIGHT_SERVO_CLOSED_POSITION : RIGHT_SERVO_OPEN_POSITION;
        return this;
    }

    /**
     * Create a task to toggle the state of the servos.
     *
     * @param servo the servo to toggle
     * @return the task
     */
    public Task toggleServoTask(ServoSide servo) {
        return new RunTask(() -> toggleServo(servo), this, true).withName("ToggleServoTask");
    }

    /**
     * Open the servos.
     *
     * @param servo the servo to open
     * @return this
     */
    public DualServos openServo(ServoSide servo) {
        if (servo == ServoSide.LEFT) {
            leftServoPosition = LEFT_SERVO_OPEN_POSITION;
            return this;
        }
        if (servo == ServoSide.RIGHT) {
            rightServoPosition = RIGHT_SERVO_OPEN_POSITION;
            return this;
        }
        leftServoPosition = LEFT_SERVO_OPEN_POSITION;
        rightServoPosition = RIGHT_SERVO_OPEN_POSITION;
        return this;
    }

    /**
     * Create a task to open the servos.
     *
     * @param servo the servo to open
     * @return the task
     */
    public Task openServoTask(ServoSide servo) {
        return new RunTask(() -> openServo(servo), this, true).withName("OpenServoTask");
    }

    /**
     * Close the servos.
     *
     * @param servo the servo to close
     * @return this
     */
    public DualServos closeServo(ServoSide servo) {
        if (servo == ServoSide.LEFT) {
            leftServoPosition = LEFT_SERVO_CLOSED_POSITION;
            return this;
        }
        if (servo == ServoSide.RIGHT) {
            rightServoPosition = RIGHT_SERVO_CLOSED_POSITION;
            return this;
        }
        leftServoPosition = LEFT_SERVO_CLOSED_POSITION;
        rightServoPosition = RIGHT_SERVO_CLOSED_POSITION;
        return this;
    }

    /**
     * Create a task to close the servos.
     *
     * @param servo the servo to close
     * @return the task
     */
    public Task closeServoTask(ServoSide servo) {
        return new RunTask(() -> closeServo(servo), this, true).withName("CloseServoTask");
    }

    /**
     * Query if a servo is open.
     *
     * @param servo the servo to query
     * @return whether the servo side is open
     */
    public boolean isOpen(ServoSide servo) {
        switch (servo) {
            case LEFT:
                return left.getPosition() == LEFT_SERVO_OPEN_POSITION;
            case RIGHT:
                return right.getPosition() == RIGHT_SERVO_OPEN_POSITION;
            case BOTH:
                return left.getPosition() == LEFT_SERVO_OPEN_POSITION && right.getPosition() == RIGHT_SERVO_OPEN_POSITION;
        }
        return false;
    }

    /**
     * Push stateful changes to the servos.
     */
    @Override
    protected void periodic() {
        left.setPosition(leftServoPosition);
        right.setPosition(rightServoPosition);
        opMode.addTelemetry("Servos: L_% R_%", left.getPosition() == LEFT_SERVO_OPEN_POSITION ? "OPEN" : "CLOSE", right.getPosition() == RIGHT_SERVO_OPEN_POSITION ? "OPEN" : "CLOSE");
    }

    /**
     * Represents the side of the servos to control.
     */
    public enum ServoSide {
        /**
         * The left servo.
         */
        LEFT,
        /**
         * The right servo.
         */
        RIGHT,
        /**
         * Both servos.
         */
        BOTH
    }
}
