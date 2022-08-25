package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp()
public class GamepadLedRumbleDemo extends OpMode {
    Gamepad.LedEffect customLedEffect;
    boolean wasDown;

    @Override
    public void init() {
        customLedEffect = new Gamepad.LedEffect.Builder().
                addStep(1.0, 0, 0, 500).
                addStep(0.0, 1.0, 0, 500).
                addStep(0.0, 0, 1.0, 500).
                setRepeating(true).
                build();

    }

    @Override
    public void loop() {
        telemetry.addLine("A to turn red");
        telemetry.addLine("B to turn green");
        telemetry.addLine("X to turn blue");
        telemetry.addLine("Y for LED pattern");
        telemetry.addLine("LB for left rumble (main)");
        telemetry.addLine("RB for right rumble (secondary)");
        telemetry.addLine("DPAD down for three blips (main)");

        if (gamepad1.a) {
            gamepad1.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (gamepad1.b) {
            gamepad1.setLedColor(0.0, 1.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (gamepad1.x) {
            gamepad1.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (gamepad1.y) {
            gamepad1.runLedEffect(customLedEffect);
        }
        if (gamepad1.left_bumper) {
            gamepad1.rumble(1.0, 0.0, 50);
        }
        if (gamepad1.right_bumper) {
            gamepad1.rumble(0.0, 1.0, 50);
        }

        if (!wasDown && gamepad1.dpad_down) {
            if (!gamepad1.isRumbling())
                gamepad1.rumbleBlips(3);
        }
        wasDown = gamepad1.dpad_down;

    }
}
