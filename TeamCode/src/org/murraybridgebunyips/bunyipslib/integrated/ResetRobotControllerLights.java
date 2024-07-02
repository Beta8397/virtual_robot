package org.murraybridgebunyips.bunyipslib.integrated;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

/**
 * Reset OpMode to clear any BunyipsOpMode-set robot controller lights.
 *
 * @author Lucas Bubner, 2024
 */
public final class ResetRobotControllerLights {
    private static boolean suppress = false;

    private ResetRobotControllerLights() {
    }

    /**
     * Call before start to suppress the OpMode from appearing in the OpMode list.
     */
    public static void suppressOpMode() {
        suppress = true;
    }

    /**
     * Register the OpMode.
     *
     * @param manager The OpModeManager to register the OpMode with.
     */
    @OpModeRegistrar
    public static void registerOpMode(OpModeManager manager) {
        if (suppress)
            return;
        manager.register(
                new OpModeMeta.Builder()
                        .setName("Reset Robot Controller Lights")
                        .setFlavor(OpModeMeta.Flavor.TELEOP)
                        // Using the FtcDashboard group to keep the OpMode out of the way
                        .setGroup("dash")
                        .build(),
                new LinearOpMode() {
                    @Override
                    public void runOpMode() {
                        hardwareMap.getAll(LynxModule.class).forEach((c) ->
                                c.setPattern(LynxModule.blinkerPolicy.getIdlePattern(c)));
                        terminateOpModeNow();
                    }
                }
        );
    }
}
