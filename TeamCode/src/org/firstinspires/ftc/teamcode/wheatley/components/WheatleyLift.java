package org.firstinspires.ftc.teamcode.wheatley.components;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.common.BunyipsComponent;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;

/**
 * Component for the arm used for lifting pixels
 *
 * @author Lachlan Paul, 2023
 * @author Lucas Bubner, 2023
 */

public class WheatleyLift extends BunyipsComponent {
    private static final double LS_OPEN = 1.0;
    private static final double LS_CLOSED = 0.0;
    private static final double RS_OPEN = 0.0;
    private static final double RS_CLOSED = 1.0;
    private static final double ARM_SPEED = 0.5;
    private final DcMotor arm;
    private final Servo leftServo;
    private final Servo rightServo;
    private int armTarget;
    private double leftClawTarget;
    private double rightClawTarget;

    public WheatleyLift(@NonNull BunyipsOpMode opMode, DcMotor arm, Servo leftServo, Servo rightServo) {
        super(opMode);
        this.arm = arm;
        this.leftServo = leftServo;
        this.rightServo = rightServo;

        // Auto close claws on init for preloading
        leftClawTarget = LS_CLOSED;
        rightClawTarget = RS_CLOSED;
        update();

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(ARM_SPEED);

        // TODO: Need limit switches to increase awareness of the arm system
    }

    /**
     * Access methods for the arm motor
     *
     * @return DcMotor instance for the arm
     */
    public DcMotor getMotor() {
        return arm;
    }

    /**
     * Used to set arm to specific position. Made for Autonomous
     *
     * @param position specific position to set the arm to
     */
    public void setPosition(int position) {
        armTarget = position;
    }

    /**
     * Used to set arm position when using a gamepad
     *
     * @param gamepadPosition the gamepad stick position to set the arm to
     */
    public void actuateUsingController(double gamepadPosition) {
        armTarget -= gamepadPosition * 2;
    }

    /**
     * Open or close the left claw
     */
    public void toggleLeftClaw() {
        if (leftServo.getPosition() == LS_OPEN) {
            leftClawTarget = LS_CLOSED;
        } else {
            leftClawTarget = LS_OPEN;
        }
    }

    /**
     * Close the left claw
     */
    public void closeLeftClaw() {
        leftClawTarget = LS_CLOSED;
    }

    /**
     * Open the left claw
     */
    public void openLeftClaw() {
        leftClawTarget = LS_OPEN;
    }

    /**
     * Open or close the right claw
     */
    public void toggleRightClaw() {
        if (rightServo.getPosition() == RS_OPEN) {
            rightClawTarget = RS_CLOSED;
        } else {
            rightClawTarget = RS_OPEN;
        }
    }

    /**
     * Close the right claw
     */
    public void closeRightClaw() {
        rightClawTarget = RS_CLOSED;
    }

    /**
     * Open the right claw
     */
    public void openRightClaw() {
        rightClawTarget = RS_OPEN;
    }

    public void update() {
        arm.setTargetPosition(armTarget);

        leftServo.setPosition(leftClawTarget);
        rightServo.setPosition(rightClawTarget);

        getOpMode().addTelemetry("Claws: L_% R_%", leftClawTarget == LS_OPEN ? "OPEN" : "CLOSED", rightClawTarget == RS_OPEN ? "OPEN" : "CLOSED");
    }
}