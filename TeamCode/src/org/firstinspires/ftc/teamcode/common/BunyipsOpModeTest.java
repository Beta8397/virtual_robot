package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Universal OpMode to test the BunyipsOpMode functionality
 */
@TeleOp(name = "BunyipsOpMode Test")
@Disabled
public class BunyipsOpModeTest extends BunyipsOpMode {
    @Override
    protected void onInit() {
        addRetainedTelemetry("======= BunyipsOpMode =======");
    }

    @Override
    protected void activeLoop() {
        assert getMovingAverageTimer() != null;
        addTelemetry(getMovingAverageTimer().toString());
        addTelemetry("GP1: %", gamepad1.toString());
        addTelemetry("GP2: %", gamepad2.toString());
    }
}
