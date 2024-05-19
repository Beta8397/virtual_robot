package org.murraybridgebunyips.bunyipslib.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
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
    // Name of the servos for telemetry
    private String NAME = "Servos";
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
     * Set the name of the servos to display in telemetry.
     *
     * @param newName the name to set
     * @return this
     */
    public DualServos withName(String newName) {
        NAME = newName;
        return this;
    }

    /**
     * Toggle the state of the servos.
     *
     * @param servo the servo to toggle
     * @return this
     */
    public DualServos toggle(ServoSide servo) {
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
    public Task toggleTask(ServoSide servo) {
        return new RunTask(() -> toggle(servo), this, true).withName("ToggleServoTask");
    }

    /**
     * Open the servos.
     *
     * @param servo the servo to open
     * @return this
     */
    public DualServos open(ServoSide servo) {
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
    public Task openTask(ServoSide servo) {
        return new RunTask(() -> open(servo), this, true).withName("OpenServoTask");
    }

    /**
     * Close the servos.
     *
     * @param servo the servo to close
     * @return this
     */
    public DualServos close(ServoSide servo) {
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
    public Task closeTask(ServoSide servo) {
        return new RunTask(() -> close(servo), this, true).withName("CloseServoTask");
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
        opMode.addTelemetry("%: L_% R_%", NAME,
                left.getPosition() == LEFT_SERVO_OPEN_POSITION ? "<font color='green'>OPEN</font>" : "<font color='red'>CLOSE</font>",
                right.getPosition() == RIGHT_SERVO_OPEN_POSITION ? "<font color='green'>OPEN</font>" : "<font color='red'>CLOSE</font>");
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
