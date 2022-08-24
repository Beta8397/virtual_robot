package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp()
public class GamepadLedDemo extends OpMode {
    Gamepad.LedEffect customLedEffect;

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

    }
}
