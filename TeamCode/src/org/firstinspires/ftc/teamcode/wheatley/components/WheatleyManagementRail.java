package org.firstinspires.ftc.teamcode.wheatley.components;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.common.BunyipsComponent;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;

/**
 * Wheatley's suspender class.
 * Making me feel like Suspender was kind of pointless.
 *
 * <p></p>
 * "Okay, listen, let me lay something on you here. It's pretty heavy.
 * They told me NEVER NEVER EVER to disengage myself from my Management Rail. Or I would DIE.
 * But we're out of options here.
 * So... get ready to catch me, alright,
 * on the off chance that I'm not dead the moment I pop off this thing."
 *
 * @author Lachlan Paul, 2023
 * @author Lucas Bubner, 2023
 */
public class WheatleyManagementRail extends BunyipsComponent {
    private static final double PWR = 1.0; // Full power since we are holding the whole robot
    private static final double ARMED = 1.0;
    private static final double OPEN = 0.0;
    private final DcMotor extension;
    // Frustration, is getting bigger,
    // bang, bang, bang,
    private final Servo /*pull my devil*/ trigger;
    private double suspenderPower;
    private double triggerTarget;

    public WheatleyManagementRail(@NonNull BunyipsOpMode opMode, DcMotor extension, Servo trigger) {
        super(opMode);
        this.extension = extension;
        this.trigger = trigger;

        triggerTarget = ARMED;

        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Assumes the motor is in the retracted position when the robot is initialised
        // This position is no longer limited because Heath will scream and get increasingly angry
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Default to locking to prevent the slider from slipping out
        suspenderPower = 0.0;
        update();
    }

    /**
     * Release the Suspender
     */
    public void activate() {
        triggerTarget = OPEN;
    }

    /**
     * Return trigger to stowed, locking position
     */
    public void reset() {
        triggerTarget = ARMED;
    }

    /**
     * Operate the suspender using a gamepad when the Suspender is released
     *
     * @param gamepadPosition gamepad.stick_y
     */
    public void actuateUsingController(double gamepadPosition) {
        if (trigger.getPosition() == OPEN) {
            // stick_y is negative on the controller
            suspenderPower = Range.clip(-gamepadPosition, -1.0, 1.0);
        }
    }

    /**
     * Send stateful updates to the hardware
     */
    public void update() {
        if (suspenderPower == 0.0) {
            // Hold arm in place, this will be the case while the arm is locked as well
            // as suspenderPower cannot be mutated while the arm is locked
            extension.setTargetPosition(extension.getCurrentPosition());
            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extension.setPower(PWR);
        } else {
            // Move arm in accordance with the user's input
            extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extension.setPower(suspenderPower);
        }
        trigger.setPosition(triggerTarget);
        getOpMode().addTelemetry("Suspender: %, % at % ticks", trigger.getPosition() == OPEN ? "OPEN" : "ARMED", triggerTarget == OPEN ? "LOCKED" : suspenderPower == 0 ? "HOLDING" : "MOVING", extension.getCurrentPosition());
    }
}
