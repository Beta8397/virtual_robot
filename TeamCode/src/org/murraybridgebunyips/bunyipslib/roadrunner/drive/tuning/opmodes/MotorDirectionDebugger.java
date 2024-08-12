package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.TriConsumer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumRoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

/**
 * This is a simple teleop routine for debugging your motor configuration.
 * Pressing each of the buttons will power its respective motor.
 * <p>
 * Button Mappings:
 * <p>
 * Xbox/PS4 Button - Motor<br>
 * X / ▢         - Front Left<br>
 * Y / Δ         - Front Right<br>
 * B / O         - Rear  Right<br>
 * A / X         - Rear  Left<p>
 * The buttons are mapped to match the wheels spatially if you
 * were to rotate the gamepad 45deg°. x/square is the front left
 * and each button corresponds to the wheel as you go clockwise
 */
public class MotorDirectionDebugger implements TriConsumer<LinearOpMode, DualTelemetry, RoadRunnerDrive> {
    /**
     * The power to run the motors at.
     */
    public double MOTOR_POWER = 0.7;

    @Override
    public void accept(LinearOpMode opMode, DualTelemetry telemetry, RoadRunnerDrive d) {
        if (!(d instanceof MecanumRoadRunnerDrive)) {
            throw new IllegalArgumentException("The MotorDirectionDebugger will only work with a MecanumRoadRunnerDrive, as it assumes there is four motors.");
        }

        MecanumRoadRunnerDrive drive = (MecanumRoadRunnerDrive) d;

        telemetry.add("Press play to begin the debugging opmode");
        telemetry.update();

        opMode.waitForStart();

        if (opMode.isStopRequested()) return;

        telemetry.clear();
        while (!opMode.isStopRequested()) {
            telemetry.add("Press each button to turn on its respective motor");
            telemetry.addNewLine();
            telemetry.add("<font face=\"monospace\">Xbox/PS4 Button - Motor</font>");
            telemetry.add("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left</font>");
            telemetry.add("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right</font>");
            telemetry.add("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Right</font>");
            telemetry.add("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Left</font>");
            telemetry.addNewLine();

            if (opMode.gamepad1.x) {
                drive.setMotorPowers(MOTOR_POWER, 0, 0, 0);
                telemetry.add("Running Motor: Front Left");
            } else if (opMode.gamepad1.y) {
                drive.setMotorPowers(0, 0, 0, MOTOR_POWER);
                telemetry.add("Running Motor: Front Right");
            } else if (opMode.gamepad1.b) {
                drive.setMotorPowers(0, 0, MOTOR_POWER, 0);
                telemetry.add("Running Motor: Rear Right");
            } else if (opMode.gamepad1.a) {
                drive.setMotorPowers(0, MOTOR_POWER, 0, 0);
                telemetry.add("Running Motor: Rear Left");
            } else {
                drive.setMotorPowers(0, 0, 0, 0);
                telemetry.add("Running Motor: None");
            }

            telemetry.update();
        }
    }
}