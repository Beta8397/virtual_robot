package org.firstinspires.ftc.teamcode.glados.components;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.common.BunyipsComponent;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.PivotMotor;
import org.firstinspires.ftc.teamcode.common.While;

import static org.firstinspires.ftc.teamcode.common.Text.round;

/**
 * Controller for the linear slider and rotation mechanism for GLaDOS.
 *
 * @author Lucas Bubner, 2023
 */
public class GLaDOSLinearSliderCore extends BunyipsComponent {
    private static final double RUNNER_POWER = 1.0;
    private static final int RUNNER_FULLY_EXTENDED_POSITION = 1000;
    private static final double ROTATOR_POWER = 1.0;

    private final PivotMotor rotator;
    private DcMotor runner;

    private double rotatorTargetAngle;
    private double extruderTargetPower;
    private double extruderTargetPercentage;

    private final While autoAlignment = new While(
            () -> runner.isBusy(),
            () -> {
                // Happening within an update-safe context
                runner.setTargetPosition((int) (extruderTargetPercentage * RUNNER_FULLY_EXTENDED_POSITION));
                runner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runner.setPower(RUNNER_POWER);
                getOpMode().addTelemetry("Aligning linear slider to % percent extension...", runner.getTargetPosition() / RUNNER_FULLY_EXTENDED_POSITION);
            },
            () -> {
                runner.setPower(0);
                runner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            },
            5 // Will be changed dynamically
    );

    public GLaDOSLinearSliderCore(@NonNull BunyipsOpMode opMode, PivotMotor rotator, DcMotor runner) {
        super(opMode);
        this.rotator = rotator;
        this.runner = runner;

        rotator.reset();
        rotator.setup();
        // Experimental
//        rotator.setSnapshot(-100.0);
        rotator.setPower(ROTATOR_POWER);

        // Default mode is set to run on tracking
        // Assumes encoder position is 0 when the arm is fully retracted
        runner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Gets the target angle of the rotator.
     *
     * @return degrees
     */
    public double getTargetAngle() {
        return rotatorTargetAngle;
    }

    /**
     * Sets the target angle of the rotator.
     *
     * @param rotatorTargetAngle degrees
     */
    public void setTargetAngle(double rotatorTargetAngle) {
        this.rotatorTargetAngle = rotatorTargetAngle;
    }


    /**
     * Delta the target angle of the rotator.
     *
     * @param gamepad_y gamepad2_left_stick_y or similar
     */
    public void setTargetAngleUsingController(double gamepad_y) {
        rotatorTargetAngle -= gamepad_y;
    }

    /**
     * Gets the current angle of the rotator.
     *
     * @return degrees
     */
    public double getAngle() {
        return rotator.getDegrees();
    }

    /**
     * Set the power of the linear slider.
     *
     * @param power power for the runner
     */
    public void setExtrusionPower(double power) {
        extruderTargetPower = Range.clip(power, -1, 1);
    }

    /**
     * Set the power of the linear slider using a controller.
     *
     * @param gamepad_y gamepad2_right_stick_y or similar
     */
    public void setExtrusionPowerUsingController(double gamepad_y) {
        extruderTargetPower = Range.clip(-gamepad_y, -1, 1);
    }

    /**
     * Autonomously adjust the power of the linear slider to reach a target position.
     * ** This will disable all manual control of the linear slider until the target position is reached. **
     *
     * @param extruderTargetPercentage target position in percentage of full extension
     */
    public void setExtrusionLength(double extruderTargetPercentage, double timeout) {
        this.extruderTargetPercentage = extruderTargetPercentage;
        autoAlignment.setTimeout(timeout);
        autoAlignment.engage();
    }

    /**
     * Send all stateful changes to the hardware.
     */
    public void update() {
        getOpMode().addTelemetry("Linear Slider Rotator: % target deg, % real deg", round(rotatorTargetAngle, 2), round(rotator.getDegrees(), 2));
        getOpMode().addTelemetry("Linear Slider Extruder: % ticks, % power", round(runner.getCurrentPosition(), 2), round(runner.getPower(), 2));
        rotator.setDegrees(rotatorTargetAngle);
        // Control autonomously if the target position is set
        if (autoAlignment.run()) return;
        runner.setPower(extruderTargetPower);
    }
}
