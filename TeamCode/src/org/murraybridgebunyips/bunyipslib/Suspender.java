package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Common component to manage arm used in CENTERSTAGE to suspend the robot during the endgame
 * This extends and rotates the arm, while ensuring the arm can only be extended if the arm is at a
 * certain position, and that it can only be retracted once it has returned to that position.
 * <p></p>
 * NOTE: This isn't actually being used, which kind of sucks.
 * We assume it's buggy, so be careful.
 *
 * @author Lucas Bubner, 2023
 * @author Lachlan Paul, 2023
 */

public class Suspender extends BunyipsComponent {

    private final PivotMotor rotation;
    private final DcMotor extension;
    private final int EXTENDED_POSITION;
    private final int RETRACTED_POSITION;
    private final double EXTENSION_POWER;
    private final double ROTATION_POWER;
    private final double STOWED_DEGREES;
    private final double OPEN_DEGREES;

    private Status status;
    private Action action;
    private boolean userStopped;

    public Suspender(@NonNull BunyipsOpMode opMode, PivotMotor rotation, DcMotor extension, int extendedPosition, int retractedPosition, double extensionPower, double rotationPower, double stowedDegrees, double openDegrees) {
        super(opMode);
        this.rotation = rotation;
        this.extension = extension;

        // Have fun constructing Suspender
        EXTENDED_POSITION = extendedPosition;
        RETRACTED_POSITION = retractedPosition;
        EXTENSION_POWER = extensionPower;
        ROTATION_POWER = rotationPower;
        STOWED_DEGREES = stowedDegrees;
        OPEN_DEGREES = openDegrees;

        // Might need to set up a limit switch to determine if the arm is down-locked
        // We will assume that the arm is down-locked for now
        status = Status.STOWED;
        action = Action.STOPPED;

        rotation.reset();
        rotation.track();

        // Sets initial values
        extension.setPower(EXTENSION_POWER);

        rotation.setDegrees(STOWED_DEGREES);
        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotation.setPower(ROTATION_POWER);
    }

    /**
     * Return the current status of the Suspender, which is determined by the position of the arm
     *
     * @return The status of the Suspender mechanism
     */
    public Status getStatus() {
        return status;
    }

    /**
     * Determine if the arm is currently stowed
     *
     * @return a boolean representing whether the arm is moving or not
     */
    public boolean isMoving() {
        return action != Action.STOPPED;
    }

    /**
     * Set Suspender from STOWED to RETRACTED
     */
    public void open() {
        // open salami
        if (status == Status.STOWED) {
            action = Action.OPENING;
        }
    }

    /**
     * Set Suspender from RETRACTED to STOWED. Cannot be called if the arm is extended.
     *
     * @see #retract()
     */
    public void close() {
        if (status != Status.EXTENDED) {
            action = Action.CLOSING;
        }
    }

    /**
     * Set Suspender from RETRACTED to EXTENDED. Cannot be called if the arm is stowed.
     *
     * @see #open()
     */
    public void extend() {
        if (status != Status.STOWED) {
            action = Action.EXTENDING;
        }
    }

    /**
     * Set Suspender from EXTENDED to RETRACTED. Cannot be called if the arm is stowed.
     *
     * @see #open()
     */
    public void retract() {
        if (status != Status.STOWED) {
            action = Action.RETRACTING;
        }
    }

    /**
     * Reset the Suspender to STOWED
     */
    public void reset() {

    }

    /**
     * Perform all motor updates queued for the Suspender system
     */
    public void update() {
        if (userStopped) return;

        switch (action) {
            case CLOSING:
                rotation.setDegrees(STOWED_DEGREES);
                break;
            case OPENING:
                rotation.setDegrees(OPEN_DEGREES);
                break;
            case STOPPED:
                rotation.setPower(0.0);
                break;
            case EXTENDING:
                extension.setTargetPosition(EXTENDED_POSITION);
                break;
            case RETRACTING:
                extension.setTargetPosition(RETRACTED_POSITION);
                break;
        }
        if (!rotation.isBusy() && !extension.isBusy()) {
            switch (action) {
                case CLOSING:
                case EXTENDING:
                    status = Status.EXTENDED;
                    break;
                case OPENING:
                case RETRACTING:
                    status = Status.RETRACTED;
                    break;
            }
            action = Action.STOPPED;
        }
    }

    /**
     * Emergency stop the Suspender system, cancelling all queued motor updates
     */
    public void interrupt() {
        userStopped = true;
        extension.setPower(0.0);
        rotation.setPower(0.0);
    }

    /**
     * Resume Suspender system motor updates after an emergency stop
     */
    public void resume() {
        userStopped = false;
        extension.setPower(EXTENSION_POWER);
        rotation.setPower(ROTATION_POWER);
    }

    /**
     * Represents all possible states the Suspender mechanism can be in
     */
    enum Status {
        STOWED,
        RETRACTED,
        EXTENDED,
        // Manual user intervention
        ERROR
    }

    /**
     * Represents all the actions the Suspender mechanism can perform.
     */
    enum Action {
        CLOSING,
        OPENING,
        RETRACTING,
        EXTENDING,
        STOPPED
    }
}
