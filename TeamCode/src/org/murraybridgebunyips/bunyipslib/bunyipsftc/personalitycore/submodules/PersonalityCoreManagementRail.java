package org.murraybridgebunyips.bunyipslib.bunyipsftc.personalitycore.submodules;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * Vertical motion motor controller for the GLaDOS/Wheatley robot.
 *
 * @author Lucas Bubner, 2023
 */
public class PersonalityCoreManagementRail extends BunyipsSubsystem {
    private static final double HOLDING_POWER = 0.3;
    private final DcMotorEx motor;
    private final ElapsedTime timer = new ElapsedTime();
    private double currentTimeout;
    private double power;

    public PersonalityCoreManagementRail(@NonNull BunyipsOpMode opMode, DcMotorEx motor) {
        super(opMode);
        this.motor = motor;
        // Assumes arm is down locked upon activation
        // If possible it would be beneficial to integrate limit switches
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void actuateUsingController(double y) {
        power = Range.clip(-y, -1.0, 1.0);
    }

    public void setPower(double p) {
        power = Range.clip(p, -1.0, 1.0);
    }

    public void runFor(double seconds, double power) {
        currentTimeout = seconds;
        this.power = power;
        timer.reset();
    }

    public boolean isBusy() {
        return currentTimeout != 0;
    }

    public void update() {
        if (currentTimeout != 0) {
            if (timer.seconds() >= currentTimeout) currentTimeout = 0;
            return;
        }
        if (power == 0.0) {
            // Hold arm in place
            motor.setTargetPosition(motor.getCurrentPosition());
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(HOLDING_POWER);
        } else {
            // Move arm in accordance with the user's input
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(power);
        }
        getOpMode().addTelemetry("Management Rail: % at % ticks", power == 0.0 ? "HOLDING" : "MOVING", motor.getCurrentPosition());
    }
}
