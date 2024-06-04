package org.murraybridgebunyips.bunyipslib.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;

/**
 * Universal OpMode to test BunyipsOpMode functionality
 */
@TeleOp(name = "BunyipsOpMode")
@Disabled
public class BunyipsOpModeTest extends BunyipsOpMode {
    @Override
    protected void onInit() {
        telemetry.addRetained("======= BunyipsOpMode =======");
    }

    @Override
    protected void activeLoop() {
        telemetry.add(getTimer().toString());
    }
}
