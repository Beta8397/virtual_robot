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

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Manual tuner that allows the user to tune the colour thresholds the camera uses to detect pixels
 * using gamepad1's left analogue stick.
 * <p></p>
 * <p>
 * Threshold Index:<br>
 * 0: lower_y<br>
 * 1: lower_cb<br>
 * 2: lower_cr<br>
 * 3: upper_y<br>
 * 4: upper_cb<br>
 * 5: upper_cr<br>
 * <p></p>
 * <p>
 * Pixel Index:<br>
 * 0: White Pixel<br>
 * 1: Purple Pixel<br>
 * 2: Yellow Pixel<br>
 * 3: Green Pixel<br>
 *
 * <p></p>
 * Controls:<br>
 * dpad_up: Increment selected pixel index<br>
 * dpad_down: Decrement selected pixel index<br>
 * dpad_left: Decrement selected Threshold index<br>
 * dpad_right: Increment selected Threshold index<br>
 * right_bumper(while held): Changes amount Threshold is changed by. Equation is left stick x and y times by 2<br>
 * right_trigger: Saves values (to telem maybe? dunno yet)
 *
 * @author Lachlan Paul, 2024
 */
@TeleOp(name = "Vision Tuner")
public class VisionTuner extends BunyipsOpMode {
    int thresholdIndex = 0;
    int pixelIndex = 0;
    double theEquation = 0;

    ColourThreshold whitePixel;
    ColourThreshold purplePixel;
    ColourThreshold yellowPixel;
    ColourThreshold greenPixel;
    ColourThreshold currentPixel;

    double lower_y;
    double lower_cb;
    double lower_cr;
    double upper_y;
    double upper_cb;
    double upper_cr;

    ArrayList<ColourThreshold> pixels;
    ArrayList<Double> scalars;

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

            // TODO: Make it so it doesn't start them all at once.
            //  It's fine for now but for performance's sake.
            vision.init(whitePixel, purplePixel, yellowPixel, greenPixel);
            vision.start(whitePixel, purplePixel, yellowPixel, greenPixel);
            vision.startPreview();

            pixels = new ArrayList<>(Arrays.asList(whitePixel, purplePixel, yellowPixel, greenPixel));
            scalars = new ArrayList<>(Arrays.asList(lower_y, lower_cb, lower_cr, upper_y, upper_cb, upper_cr));
            currentPixel = whitePixel;  // Set it to white pixel by default
        } catch (IllegalArgumentException e) {
            throw new EmergencyStop("VisionTest is missing a webcam called 'webcam'!");
        }
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
            if (pixelIndex == 3) {
                pixelIndex = 0;
            } else {
                pixelIndex++;
            }
        }

        if (gamepad1.dpad_down && !upPressed && !downPressed) {
            if (pixelIndex == 0) {
                pixelIndex = 3;
            } else {
                pixelIndex--;
            }
        }

        if (gamepad1.dpad_left && !leftPressed && !rightPressed) {
            if (thresholdIndex == 0) {
                thresholdIndex = 5;
            } else {
                thresholdIndex--;
            }
        }

        if (gamepad1.dpad_right && !leftPressed && !rightPressed) {
            if (thresholdIndex == 5) {
                thresholdIndex = 0;
            } else {
                thresholdIndex++;
            }
        }

        // NOTE: If this doesn't work, I have a backup of the original match case string with fixed
        // index numbers.
        currentPixel = pixels.get(pixelIndex);
        scalars.set(thresholdIndex, scalars.get(thresholdIndex) + theEquation);

        upPressed = gamepad1.dpad_up;
        downPressed = gamepad1.dpad_down;
        leftPressed = gamepad1.dpad_left;
        rightPressed = gamepad1.dpad_right;

        currentPixel.setLower(new Scalar(lower_y, lower_cb, lower_cr));
        currentPixel.setUpper(new Scalar(upper_y, upper_cb, upper_cr));

        addTelemetry("Current Threshold Index: %", thresholdIndex);
        addTelemetry("Current Pixel Index: %", pixelIndex);
        addTelemetry("Current Value to Change Threshold By: %", theEquation);

        addTelemetry("\nThreshold Index Key:");
        addTelemetry("0: lower_y\n1: lower_cb\n2: lower_cr\n3: upper_y\n4: upper_cb\n5: upper_cr\n");

        addTelemetry("Pixel Index Key");
        addTelemetry("0: White Pixel\n1: Purple Pixel\n2: Yellow Pixel\n3: Green Pixel");
    }
}
