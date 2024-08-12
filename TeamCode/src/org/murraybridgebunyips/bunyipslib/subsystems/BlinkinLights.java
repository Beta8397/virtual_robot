package org.murraybridgebunyips.bunyipslib.subsystems;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.tasks.RunForTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

public class BlinkinLights extends BunyipsSubsystem {
    private final RevBlinkinLedDriver lights;
    private final RevBlinkinLedDriver.BlinkinPattern defaultPattern;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern;

    public BlinkinLights(RevBlinkinLedDriver light, RevBlinkinLedDriver.BlinkinPattern defaultBlinkPattern) {
        lights = light;
        defaultPattern = defaultBlinkPattern;
        currentPattern = defaultBlinkPattern;

        lights.setPattern(defaultPattern);
    }

    public Task setPatternForTask(Measure<Time> duration, RevBlinkinLedDriver.BlinkinPattern pattern) {
        return new RunForTask(duration, () -> currentPattern = pattern, this::resetPattern)
                .onSubsystem(this, true)
                .withName("Lights:" + pattern.name());
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern() {
        return currentPattern;
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        currentPattern = pattern;
    }

    public void resetPattern() {
        currentPattern = defaultPattern;
    }

    public void turnOff() {
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
        opMode.telemetry.add("Current Pattern:" + currentPattern.name());
        lights.setPattern(currentPattern);
    }
}
