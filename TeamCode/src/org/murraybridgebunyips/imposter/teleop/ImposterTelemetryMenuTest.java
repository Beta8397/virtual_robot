package org.murraybridgebunyips.imposter.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.external.TelemetryMenu;
import org.murraybridgebunyips.bunyipslib.external.TelemetryMenu.StaticClickableOption;

@TeleOp
public class ImposterTelemetryMenuTest extends BunyipsOpMode {
    TelemetryMenu.MenuElement root = new TelemetryMenu.MenuElement("main", true);
    TelemetryMenu.MenuElement sub = new TelemetryMenu.MenuElement("sub", false);
    StaticClickableOption o = new StaticClickableOption("a") {
        @Override
        protected void onClick() {
            Dbg.log("clicked!");
        }
    };
    TelemetryMenu.StaticItem o2 = new TelemetryMenu.StaticItem("THING");

    private TelemetryMenu menu;

    @Override
    protected void onInit() {
        menu = new TelemetryMenu(telemetry, root);
        root.addChild(o);
        root.addChild(o2);
        root.addChild(sub);
        sub.addChild(o);
    }

    @Override
    protected void activeLoop() {
        menu.loop(gamepad1);
    }
}
