package org.murraybridgebunyips.bunyipslib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * BunyipsOpMode Kotlin and Java interop class to hide the SDK fields from the user, and replace them with the custom
 * BunyipsLib interfaces. This class is not intended to be used by the user, and is only used to provide
 * the BunyipsOpMode interface to the user.
 *
 * @author Lucas Bubner, 2024
 * @noinspection unused
 * @see BunyipsOpMode
 */
public class BOMInternal extends LinearOpMode {
    // Override fields that will be supplemented by BunyipsOpMode
    // This interop problem will probably be fixed in Kotlin v2.0.0, but for now we need to do this so we can override Java fields from superclasses.
    // https://youtrack.jetbrains.com/issue/KT-55017/Prioritize-Kotlin-property-from-derived-class-over-Java-field-from-base-class
    private final Gamepad gamepad1 = null;
    private final Gamepad gamepad2 = null;
    private final Telemetry telemetry = null;

    // Prevent users from using this class
    BOMInternal() {
    }

    // For use in BunyipsOpMode to assign custom objects
    final Gamepad getSdkGamepad1() {
        return super.gamepad1;
    }

    final Gamepad getSdkGamepad2() {
        return super.gamepad2;
    }

    final Telemetry getSdkTelemetry() {
        return super.telemetry;
    }

    /**
     * Main BunyipsOpMode executor method.
     */
    @SuppressWarnings("RedundantThrows")
    protected void runBunyipsOpMode() throws InterruptedException {
        // To be overridden by BunyipsOpMode, which can throw an InterruptedException
    }

    /**
     * Main OpMode thread method.
     * You may choose to override this method if you require a hook on the OpMode thread,
     * but ensure to call {@link #runBunyipsOpMode()} in your override to start the BunyipsOpMode lifecycle.
     *
     * @throws InterruptedException if the OpMode is interrupted
     */
    @Override
    public void runOpMode() throws InterruptedException {
        runBunyipsOpMode();
    }
}
