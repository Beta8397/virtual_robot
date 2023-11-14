package org.firstinspires.ftc.teamcode.glados.components;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.common.BunyipsComponent;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;

import static org.firstinspires.ftc.teamcode.common.Text.round;

/**
 * Controller for the self-aligning servo mechanism for GLaDOS.
 *
 * @author Lucas Bubner, 2023
 */
public class GLaDOSAlignmentCore extends BunyipsComponent {
    /**
     * Assumes the alignment servo has range 0-1 where 0 == as far back as the arm lets and 1 == bottom is facing forward.
     */
    private final Servo alignment;

    /**
     * Angle at which the alignment servo is trying to align to when the arm is up.
     */
    private double targetAngle;

    /**
     * Angle at which the rotator is considered to be in the up position and the alignment servo
     * can begin aligning to the targetAngle.
     */
    private double downLockThresholdAngle;

    /**
     * Manual mode angle
     */
    private double userPosition;

    private Mode mode;

    public GLaDOSAlignmentCore(@NonNull BunyipsOpMode opMode, Servo alignment, Mode mode, double targetAngle, double downLockThresholdAngle) {
        super(opMode);
        this.alignment = alignment;
        this.targetAngle = targetAngle;
        this.downLockThresholdAngle = downLockThresholdAngle;
        this.mode = mode;

        // Align to down position
        alignment.setPosition(1.0);
        userPosition = 1.0;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public double getDownLockThresholdAngle() {
        return downLockThresholdAngle;
    }

    public void setDownLockThresholdAngle(double downLockThresholdAngle) {
        this.downLockThresholdAngle = downLockThresholdAngle;
    }

    /**
     * Manual mode, use controller dpad to adjust alignment servo position.
     *
     * @param up   gamepad2.dpad_up
     * @param down gamepad2.dpad_down
     */
    public void setPositionUsingDpad(boolean up, boolean down) {
        if (up) {
            userPosition -= 0.01;
        }
        if (down) {
            userPosition += 0.01;
        }
        userPosition = Range.clip(userPosition, 0, 1);
    }

    /**
     * Updates the alignment servo in response to the current angle of the arm.
     * Usable in AUTO mode.
     *
     * @param theta degrees of the arm
     */
    public void update(double theta) {
        if (mode != Mode.AUTO) {
            // Silently reject theta and use manual update
            update();
            return;
        }
        if (theta < downLockThresholdAngle) {
            alignment.setPosition(convertDegreesToServoPosition(targetAngle));
            return;
        }
        alignment.setPosition(0.0);
        getOpMode().addTelemetry("Alignment (AUTO): % target", round(alignment.getPosition(), 2));
    }

    /**
     * Update the alignment servo based on user input.
     * Usable in MANUAL mode.
     */
    public void update() {
        if (mode != Mode.MANUAL) {
            // noop, we need theta to use automatic mode
            return;
        }
        alignment.setPosition(userPosition);
        getOpMode().addTelemetry("Alignment (MAN): % target", round(alignment.getPosition(), 2));
    }

    /**
     * Converts degrees to a servo position.
     *
     * @param degrees degrees to convert
     * @return 0-1 servo position
     */
    private double convertDegreesToServoPosition(double degrees) {
        return degrees / 180.0;
    }

    public Mode getMode() {
        return mode;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public enum Mode {
        AUTO,
        MANUAL
    }
}
