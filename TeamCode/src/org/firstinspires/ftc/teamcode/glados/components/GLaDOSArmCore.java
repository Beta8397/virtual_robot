package org.firstinspires.ftc.teamcode.glados.components;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.common.BunyipsComponent;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.NullSafety;
import org.firstinspires.ftc.teamcode.common.PivotMotor;

/**
 * Pixel Movement & Suspension arm mechanism
 *
 * @author Lucas Bubner, 2023
 */
public class GLaDOSArmCore extends BunyipsComponent {
    /**
     * When the arm is above the ground (where the alignment will not automatically reset to 0.0)
     * the alignment servo should be at this target angle (up from ground anticlockwise).
     */
    protected static final double SERVO_ON_ELEVATION_TARGET_ANGLE = 30.0;
    /**
     * When the arm is below this angle (down from ground clockwise) the alignment servo should
     * lock to 0.0.
     */
    protected static final double SERVO_ON_ELEVATION_DOWN_LOCK_THRESHOLD_ANGLE = 10.0;
    private GLaDOSAlignmentCore alignmentController;
    private GLaDOSServoCore servoController;
    private GLaDOSLinearSliderCore sliderController;

    public GLaDOSArmCore(@NonNull BunyipsOpMode opMode, PivotMotor rotator, DcMotor extensionRunner, Servo alignment, Servo left, Servo right, GLaDOSAlignmentCore.Mode alignmentMode) {
        super(opMode);
        if (NullSafety.assertComponentArgs(opMode, GLaDOSLinearSliderCore.class, rotator, extensionRunner))
            sliderController = new GLaDOSLinearSliderCore(opMode, rotator, extensionRunner);

        if (NullSafety.assertComponentArgs(opMode, GLaDOSAlignmentCore.class, alignment))
            alignmentController = new GLaDOSAlignmentCore(opMode, alignment, alignmentMode, SERVO_ON_ELEVATION_TARGET_ANGLE, SERVO_ON_ELEVATION_DOWN_LOCK_THRESHOLD_ANGLE);

        if (NullSafety.assertComponentArgs(opMode, GLaDOSServoCore.class, left, right))
            servoController = new GLaDOSServoCore(opMode, left, right);
    }

    public GLaDOSServoCore getServoController() {
        return servoController;
    }

    public GLaDOSLinearSliderCore getSliderController() {
        return sliderController;
    }

    public GLaDOSAlignmentCore getAlignmentController() {
        return alignmentController;
    }

    public void update() {
        // Push stateful changes from any changes to these controllers
        servoController.update();
        sliderController.update();

        // Alignment servo is fully handled automatically by the alignment controller
        alignmentController.update(sliderController.getAngle());
    }
}
