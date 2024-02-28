package org.murraybridgebunyips.bunyipslib.vision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;
import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.GreenPixel;
import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.PurplePixel;
import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.WhitePixel;
import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.YellowPixel;
import org.opencv.core.Scalar;

@TeleOp(name = "Vision Tuner")
public class VisionTuner extends BunyipsOpMode {
    int thresholdIndex = 1;
    int pixelIndex = 1;
    double theEquation = 0;

    ColourThreshold whitePixel;
    ColourThreshold purplePixel;
    ColourThreshold yellowPixel;
    ColourThreshold greenPixel;
    ColourThreshold[] pixels;
    ColourThreshold currentPixel;

    double lower_y;
    double lower_cb;
    double lower_cr;
    double upper_y;
    double upper_cb;
    double upper_cr;


    private boolean upPressed;
    private boolean downPressed;
    private boolean leftPressed;
    private boolean rightPressed;

    @Override
    protected void onInit() {
        try {
            WebcamName webcam = (WebcamName) hardwareMap.get("webcam");
            Vision vision = new Vision(webcam);
            whitePixel = new WhitePixel();
            purplePixel = new PurplePixel();
            yellowPixel = new YellowPixel();
            greenPixel = new GreenPixel();

            // This hasn't been used thus far, and it probably won't be.
            // But just in case we're keeping it here.
            pixels = new ColourThreshold[]{whitePixel, purplePixel, yellowPixel, greenPixel};
            currentPixel = whitePixel;  // Set it to white pixel by default
        } catch (IllegalArgumentException e) {
            throw new EmergencyStop("VisionTest is missing a webcam called 'webcam'!");
        }

        addTelemetry("Threshold Index Key:");
        addTelemetry("1: lower_y\n2: lower_cb\n3: lower_cr\n4: upper_y\n5: upper_cb\n6: upper_cr\n");

        addTelemetry("Pixel Index Key");
        addTelemetry("1: White Pixel\n2: Purple Pixel\n3: Yellow Pixel\n4: Green Pixel");
    }

    @Override
    protected void activeLoop() {
        // FIXME: This code is crap code, hopefully temp code.
        //  It's hard to expand upon, and could be optimised

        // TODO:
        //  The value and the variable it was currently writing to should be shown in telemetry.

        if (gamepad1.right_bumper) {
            theEquation = (gamepad1.left_stick_y + gamepad1.left_stick_x) * 2;
        }

        if (gamepad1.dpad_up && !upPressed && !downPressed) {
            if (pixelIndex == 1) {
                pixelIndex = 4;
            } else {
                pixelIndex--;
            }
        }

        if (gamepad1.dpad_down && !upPressed && !downPressed) {
            if (pixelIndex == 4) {
                pixelIndex = 1;
            } else {
                pixelIndex++;
            }
        }

        if (gamepad1.dpad_left && !leftPressed && !rightPressed) {
            if (thresholdIndex == 1) {
                thresholdIndex = 6;
            } else {
                thresholdIndex--;
            }
        }

        if (gamepad1.dpad_right && !leftPressed && !rightPressed) {
            if (thresholdIndex == 6) {
                thresholdIndex = 1;
            } else {
                thresholdIndex++;
            }
        }

        switch (pixelIndex) {
            case 1:
                currentPixel = whitePixel;
                break;
            case 2:
                currentPixel = purplePixel;
                break;
            case 3:
                currentPixel = yellowPixel;
                break;
            case 4:
                currentPixel = greenPixel;
        }

        switch (thresholdIndex) {
            case 1:
                lower_y += theEquation;
                break;
            case 2:
                lower_cb += theEquation;
                break;
            case 3:
                lower_cr += theEquation;
                break;
            case 4:
                upper_y += theEquation;
                break;
            case 5:
                upper_cb += theEquation;
                break;
            case 6:
                upper_cr += theEquation;
                break;
        }

        upPressed = gamepad1.dpad_up;
        downPressed = gamepad1.dpad_down;
        leftPressed = gamepad1.dpad_left;
        rightPressed = gamepad1.dpad_right;

        currentPixel.setLower(new Scalar(lower_y, lower_cb, lower_cr));
        currentPixel.setUpper(new Scalar(upper_y, upper_cb, upper_cr));

        addTelemetry("Current Threshold Index: %", thresholdIndex);
        addTelemetry("Current Pixel Index: %", pixelIndex);
        addTelemetry("Current Gamepad Thing: %", theEquation);
    }
}
