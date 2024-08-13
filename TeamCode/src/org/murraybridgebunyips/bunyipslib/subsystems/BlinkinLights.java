package org.murraybridgebunyips.bunyipslib.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.tasks.IdleTask;
import org.murraybridgebunyips.bunyipslib.tasks.RunForTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * LED driver subsystem for the REV Blinkin Lights. Able to integrate with the Task system to schedule lighting patterns.
 *
 * @author Lachlan Paul, 2024
 * @author Lucas Bubner, 2024
 */
public class BlinkinLights extends BunyipsSubsystem {
    /**
     * Tasks for BlinkinLights.
     */
    public final Tasks tasks = new Tasks();

    private final RevBlinkinLedDriver lights;
    private final RevBlinkinLedDriver.BlinkinPattern defaultPattern;

    private RevBlinkinLedDriver.BlinkinPattern currentPattern;

    /**
     * Create a new BlinkinLights subsystem.
     *
     * @param lights         the LED driver to use.
     * @param defaultPattern the default pattern which will be the one this driver goes back to as a default.
     */
    public BlinkinLights(RevBlinkinLedDriver lights, RevBlinkinLedDriver.BlinkinPattern defaultPattern) {
        this.lights = lights;
        this.defaultPattern = defaultPattern;
        currentPattern = defaultPattern;

        lights.setPattern(this.defaultPattern);
    }

    /**
     * Get the current pattern.
     *
     * @return currently respected pattern
     */
    public RevBlinkinLedDriver.BlinkinPattern getPattern() {
        return currentPattern;
    }

    /**
     * Set the current pattern. Will no-op if a task is running on this subsystem.
     *
     * @param pattern the pattern to update to.
     */
    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (getCurrentTask() instanceof IdleTask)
            currentPattern = pattern;
    }

    /**
     * Reset the pattern back to the default. Will no-op if a task is running on this subsystem.
     */
    public void resetPattern() {
        if (getCurrentTask() instanceof IdleTask)
            currentPattern = defaultPattern;
    }

    /**
     * Cancel tasks and turn off the lights. Auto-called on subsystem disable.
     */
    public void turnOff() {
        cancelCurrentTask();
        currentPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    }

    @Override
    protected void onDisable() {
        turnOff();
    }

    @Override
    protected void onEnable() {
        currentPattern = defaultPattern;
    }

    @Override
    protected void periodic() {
        opMode.telemetry.add("%: Pattern->%", name, currentPattern.name()).color("gray");
        lights.setPattern(currentPattern);
    }

    /**
     * Tasks for BlinkinLights, access with {@link #tasks}.
     */
    public class Tasks {
        /**
         * Set the current pattern for a set duration.
         *
         * @param duration the time duration
         * @param pattern  the pattern to set
         * @return a task to set the pattern for a duration
         */
        public Task setPatternFor(Measure<Time> duration, RevBlinkinLedDriver.BlinkinPattern pattern) {
            return new RunForTask(duration, () -> {
                if (getCurrentTask() instanceof IdleTask) currentPattern = pattern;
            }, BlinkinLights.this::resetPattern)
                    .onSubsystem(BlinkinLights.this, true)
                    .withName("Lights:" + pattern.name());
        }
    }
}
