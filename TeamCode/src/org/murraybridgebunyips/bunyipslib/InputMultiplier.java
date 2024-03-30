package org.murraybridgebunyips.bunyipslib;


import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * Multi-purpose input sensitivity multiplier with increment and decrement.
 *
 * @author Lucas Bubner, 2024
 */
public class InputMultiplier extends BunyipsSubsystem {
    private final double[] multipliers;
    private String name = null;
    private int index = 0;

    /**
     * Create a new InputMultiplier.
     *
     * @param multipliers the multipliers to use, when the index is selected the corresponding multiplier is used
     */
    public InputMultiplier(double... multipliers) {
        this.multipliers = multipliers;
    }

    /**
     * Set the name of the InputMultiplier to display in telemetry.
     *
     * @param displayName the name to set
     * @return this
     */
    public InputMultiplier withName(String displayName) {
        name = displayName;
        return this;
    }

    /**
     * Set the default index of the multiplier table.
     *
     * @param defaultIndex the index to set
     * @return this
     */
    public InputMultiplier withDefaultIndex(int defaultIndex) {
        if (defaultIndex < 0 || defaultIndex >= multipliers.length)
            throw new EmergencyStop("Default index out of bounds");
        index = defaultIndex;
        return this;
    }

    /**
     * Increment the multiplier table index.
     */
    public void increment() {
        if (index >= multipliers.length - 1) return;
        index++;
    }

    /**
     * Create a task to increment the multiplier table index.
     *
     * @return the task
     */
    public Task incrementTask() {
        return new RunTask(this::increment, this, false).withName("IncrementMultiplier");
    }

    /**
     * Decrement the multiplier table index.
     */
    public void decrement() {
        if (index <= 0) return;
        index--;
    }

    /**
     * Create a task to decrement the multiplier table index.
     *
     * @return the task
     */
    public Task decrementTask() {
        return new RunTask(this::decrement, this, false).withName("DecrementMultiplier");
    }

    /**
     * Set the multiplier table index.
     *
     * @param index the index to set
     */
    public void set(int index) {
        if (index < 0 || index >= multipliers.length) {
            throw new IllegalArgumentException("Index out of bounds");
        }
        this.index = index;
    }

    /**
     * Get the applied multiplier from the index.
     */
    public double getMultiplier() {
        return multipliers[index];
    }

    /**
     * Optionally update the telemetry with the current multiplier.
     */
    @Override
    protected void periodic() {
        opMode.addTelemetry(
                "%: %x (%/%)",
                name != null ? "Mul-" + name : "Input Multiplier",
                multipliers[index],
                index + 1,
                multipliers.length
        );
    }
}
