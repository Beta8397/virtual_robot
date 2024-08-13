package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning;

import static org.murraybridgebunyips.bunyipslib.Text.html;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.TriConsumer;
import org.murraybridgebunyips.bunyipslib.external.TelemetryMenu;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.AutomaticFeedforwardTuner;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.BackAndForth;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.DriveVelocityPIDTuner;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.FollowerPIDTuner;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.LocalizationTest;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.ManualFeedforwardTuner;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.MaxAngularVeloTuner;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.MaxVelocityTuner;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.MotorDirectionDebugger;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.SplineTest;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.StrafeTest;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.StraightTest;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.TrackWidthTuner;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.TrackingWheelForwardOffsetTuner;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.TrackingWheelLateralDistanceTuner;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes.TurnTest;

import java.util.Objects;

/**
 * Utility OpMode you will extend to enable an OpMode to tune your RoadRunner coefficients and parameters. This uses
 * Telemetry to show a menu of tuning options, with static fields being adjusted here, essentially collecting all
 * RoadRunner tuning into one OpMode. You may choose to run the OpModes yourself as they consume the instances you
 * provide, but using this class makes it more straightforward and has built-in dashboard tuning. It is not recommended
 * you run the tuning OpModes yourself, and instead run them using this class.
 * <p>
 * To use this class, extend it, supply an appropriately configured instance of your base RoadRunnerDrive (not the BunyipsLib
 * version), and ensure the localizer you wish to use has been set for it. Then, treat is like a normal OpMode
 * ({@link TeleOp} annotation). You can adjust all the constants used in testing via FtcDashboard under RoadRunnerTuning.
 * <p>
 * Telemetry is mirrored between FtcDashboard and the Driver Station using a {@link DualTelemetry} instance, the same
 * used in {@link BunyipsOpMode}. It is not required that a Driver Station be active for tuning.
 * <p>
 * To look at details of a tuning process, you can find them attached to the classes under the ./opmodes package,
 * and by applying the tuning process for RoadRunner v0.5.
 *
 * @author Lucas Bubner, 2024
 */
@Config
public abstract class RoadRunnerTuning extends LinearOpMode {
    // Tuning procedures are exposed as public static fields so FtcDashboard can change internally public properties,
    // with documentation of their operation being attached to their actual classes, not these instances.
    // These tests are ordered in the general tuning process order, minus LocalizationTest which is usually run whenever.
    /**
     * Debug mode to determine which motors on a four-wheel setup are attached where.
     */
    public static MotorDirectionDebugger MotorDirectionDebugger = new MotorDirectionDebugger();
    /**
     * Determines maximum angular velocity of the robot.
     */
    public static MaxAngularVeloTuner MaxAngularVeloTuner = new MaxAngularVeloTuner();
    /**
     * Determines maximum velocity of the robot.
     */
    public static MaxVelocityTuner MaxVelocityTuner = new MaxVelocityTuner();
    /**
     * For three-wheel odometry, tuner to find the lateral offset coefficient.
     */
    public static TrackingWheelLateralDistanceTuner TrackingWheelLateralOffsetTuner = new TrackingWheelLateralDistanceTuner();
    /**
     * For three-wheel odometry, tuner to find the forward offset coefficient.
     */
    public static TrackingWheelForwardOffsetTuner TrackingWheelForwardOffsetTuner = new TrackingWheelForwardOffsetTuner();
    /**
     * General drive capabilities, to test field localization and odometry pod tracking.
     */
    public static LocalizationTest LocalizationTest = new LocalizationTest();
    /**
     * Feedforward coefficient automated calculation to get a general idea of the FF coefficients, not fully accurate.
     */
    public static AutomaticFeedforwardTuner AutomaticFeedforwardTuner = new AutomaticFeedforwardTuner();
    /**
     * Most important tuning procedure to tune precise feedforward coefficients. Informally known as the vroom-vroom test.
     */
    public static ManualFeedforwardTuner ManualFeedforwardTuner = new ManualFeedforwardTuner();
    /**
     * PID tuner for velocity mode (note: not recommended to use velocity PID).
     */
    public static DriveVelocityPIDTuner DriveVelocityPIDTuner = new DriveVelocityPIDTuner();
    /**
     * Forward test to verify x direction localization.
     */
    public static StraightTest StraightTest = new StraightTest();
    /**
     * Strafing test to verify y direction localization.
     */
    public static StrafeTest StrafeTest = new StrafeTest();
    /**
     * Tuner to find the effective track width coefficient.
     */
    public static TrackWidthTuner TrackWidthTuner = new TrackWidthTuner();
    /**
     * Rotation test to verify z-axis localization.
     */
    public static TurnTest TurnTest = new TurnTest();
    /**
     * Follower PID test (moves in a square forever). Also known as the Square Test.
     */
    public static FollowerPIDTuner FollowerPIDTuner = new FollowerPIDTuner();
    /**
     * Follower PID test (moves forward and backward forever).
     */
    public static BackAndForth BackAndForth = new BackAndForth();
    /**
     * Final test of coefficients with an advanced spline path.
     */
    public static SplineTest SplineTest = new SplineTest();

    @NonNull
    protected abstract RoadRunnerDrive getBaseRoadRunnerDrive();

    @Override
    @SuppressWarnings("unchecked")
    public final void runOpMode() {
        DualTelemetry out = new DualTelemetry(this, null, "RR Tuning");
        RoadRunnerDrive drive = getBaseRoadRunnerDrive();
        if (drive instanceof BunyipsSubsystem) {
            throw new IllegalArgumentException("Drive supplied to getBaseRoadRunnerDrive() is a BunyipsSubsystem. You must pass a base RoadRunner drive and set the localizer appropriate to your drive with setLocalizer(). This is to ensure the base drive is working to eliminate any potential issues early at their source.");
        }
        Objects.requireNonNull(drive, "getBaseRoadRunnerDrive() returned null");

        TelemetryMenu.MenuElement root = new TelemetryMenu.MenuElement("Tuning OpMode Selection", true);
        // Cannot create an array of generic types, will cast later and assert these are safe
        Object[] modes = {
                MotorDirectionDebugger,
                MaxAngularVeloTuner,
                MaxVelocityTuner,
                TrackingWheelLateralOffsetTuner,
                TrackingWheelForwardOffsetTuner,
                LocalizationTest,
                AutomaticFeedforwardTuner,
                ManualFeedforwardTuner,
                DriveVelocityPIDTuner,
                StraightTest,
                StrafeTest,
                TrackWidthTuner,
                TurnTest,
                FollowerPIDTuner,
                BackAndForth,
                SplineTest
        };
        TelemetryMenu.StaticClickableOption[] opModes = new TelemetryMenu.StaticClickableOption[modes.length];
        // Have to use array access due to inner class variable mutation
        Object[] selection = {null};
        for (int i = 0; i < modes.length; i++) {
            // Must be considered final as it is used in the inner class
            int finalI = i;
            opModes[i] = new TelemetryMenu.StaticClickableOption(modes[finalI].getClass().getSimpleName()) {
                @Override
                protected void onClick() {
                    selection[0] = modes[finalI];
                }
            };
        }
        root.addChildren(opModes);
        TelemetryMenu menu = new TelemetryMenu(out, root);

        while (selection[0] == null && opModeInInit()) {
            menu.loop(gamepad1);
            out.add("Select an option above to run tuning for using gamepad1. Restart the OpMode to pick a different mode.");
            out.update();
        }

        out.clearAll();
        if (selection[0] == null) return;

        // Defer the OpMode to the tuning OpMode now, it is confirmed to be safe to cast
        out.setOpModeStatus(html().bold(selection[0].getClass().getSimpleName()).toString());
        ((TriConsumer<LinearOpMode, DualTelemetry, RoadRunnerDrive>) selection[0])
                .accept(this, out, drive);
    }
}
