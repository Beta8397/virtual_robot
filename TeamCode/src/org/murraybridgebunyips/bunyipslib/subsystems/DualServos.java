package org.murraybridgebunyips.bunyipslib.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * Control a set of two servos together.
 */
public class DualServos extends BunyipsSubsystem {
    /**
     * Tasks for DualServos.
     */
    public final Tasks tasks = new Tasks();
    private final double LEFT_SERVO_CLOSED_POSITION;
    private final double LEFT_SERVO_OPEN_POSITION;
    private final double RIGHT_SERVO_CLOSED_POSITION;
    private final double RIGHT_SERVO_OPEN_POSITION;
    private Servo left;
    private Servo right;
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
        if (leftClosed == leftOpen || rightClosed == rightOpen)
            throw new IllegalArgumentException("Open and close positions for either servo cannot be the same");

        LEFT_SERVO_CLOSED_POSITION = leftClosed;
        LEFT_SERVO_OPEN_POSITION = leftOpen;
        RIGHT_SERVO_CLOSED_POSITION = rightClosed;
        RIGHT_SERVO_OPEN_POSITION = rightOpen;

        if (!assertParamsNotNull(left, right)) return;
        this.left = left;
        this.right = right;

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
        opMode.telemetry.add("%: Left->% Right->%", name,
                left.getPosition() == LEFT_SERVO_OPEN_POSITION ? "<font color='green'>OPEN</font>" : "<font color='yellow'>CLOSE</font>",
                right.getPosition() == RIGHT_SERVO_OPEN_POSITION ? "<font color='green'>OPEN</font>" : "<font color='yellow'>CLOSE</font>");
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

    /**
     * DualServo tasks, access with {@link #tasks}.
     */
    public class Tasks {
        /**
         * Create a task to open the left servo.
         *
         * @return the task
         */
        public Task openLeft() {
            return new RunTask(() -> open(ServoSide.LEFT))
                    .onSubsystem(DualServos.this, true)
                    .withName("Open:LEFT");
        }

        /**
         * Create a task to open the right servo.
         *
         * @return the task
         */
        public Task openRight() {
            return new RunTask(() -> open(ServoSide.RIGHT))
                    .onSubsystem(DualServos.this, true)
                    .withName("Open:RIGHT");
        }

        /**
         * Create a task to open both servos.
         *
         * @return the task
         */
        public Task openBoth() {
            return new RunTask(() -> {
                open(ServoSide.LEFT);
                open(ServoSide.RIGHT);
            })
                    .onSubsystem(DualServos.this, true)
                    .withName("Open:BOTH");
        }

        /**
         * Create a task to close the left servo.
         *
         * @return the task
         */
        public Task closeLeft() {
            return new RunTask(() -> close(ServoSide.LEFT))
                    .onSubsystem(DualServos.this, true)
                    .withName("Close:LEFT");
        }

        /**
         * Create a task to close the right servo.
         *
         * @return the task
         */
        public Task closeRight() {
            return new RunTask(() -> close(ServoSide.RIGHT))
                    .onSubsystem(DualServos.this, true)
                    .withName("Close:RIGHT");
        }

        /**
         * Create a task to close both servos.
         *
         * @return the task
         */
        public Task closeBoth() {
            return new RunTask(() -> {
                close(ServoSide.LEFT);
                close(ServoSide.RIGHT);
            })
                    .onSubsystem(DualServos.this, true)
                    .withName("Close:BOTH");
        }

        /**
         * Create a task to toggle the left servo.
         *
         * @return the task
         */
        public Task toggleLeft() {
            return new RunTask(() -> toggle(ServoSide.LEFT))
                    .onSubsystem(DualServos.this, true)
                    .withName("Toggle:LEFT");
        }

        /**
         * Create a task to toggle the right servo.
         *
         * @return the task
         */
        public Task toggleRight() {
            return new RunTask(() -> toggle(ServoSide.RIGHT))
                    .onSubsystem(DualServos.this, true)
                    .withName("Toggle:RIGHT");
        }

        /**
         * Create a task to toggle both servos.
         *
         * @return the task
         */
        public Task toggleBoth() {
            return new RunTask(() -> {
                toggle(ServoSide.LEFT);
                toggle(ServoSide.RIGHT);
            })
                    .onSubsystem(DualServos.this, true)
                    .withName("Toggle:BOTH");
        }
    }
}
