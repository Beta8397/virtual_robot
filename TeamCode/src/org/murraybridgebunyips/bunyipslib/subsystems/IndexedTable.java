package org.murraybridgebunyips.bunyipslib.subsystems;


import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * Multi-purpose index-based table with increment and decrement functionality.
 *
 * @author Lucas Bubner, 2024
 */
public class IndexedTable extends BunyipsSubsystem {
    private final double[] tableValues;
    // Name of the table for telemetry, default "Index Table"
    private String NAME = null;
    private int index = 0;

    /**
     * Create a new IndexedTable.
     *
     * @param tableValues the values to use, when the index is selected the corresponding value is used
     */
    public IndexedTable(double... tableValues) {
        this.tableValues = tableValues;
    }

    /**
     * Set the name of the IndexedTable to display in telemetry.
     *
     * @param displayName the name to set
     * @return this
     */
    public IndexedTable withName(String displayName) {
        NAME = displayName;
        return this;
    }

    /**
     * Set the default index of the indexed table.
     *
     * @param defaultIndex the index to set
     * @return this
     */
    public IndexedTable withDefaultIndex(int defaultIndex) {
        if (defaultIndex < 0 || defaultIndex >= tableValues.length)
            throw new EmergencyStop("Default index out of bounds");
        index = defaultIndex;
        return this;
    }

    /**
     * Increment the table index.
     */
    public void increment() {
        if (index >= tableValues.length - 1) return;
        index++;
    }

    /**
     * Create a task to increment the table index.
     *
     * @return the task
     */
    public Task incrementTask() {
        return new RunTask(this::increment, this, false).withName("Increment Index");
    }

    /**
     * Decrement the table index.
     */
    public void decrement() {
        if (index <= 0) return;
        index--;
    }

    /**
     * Create a task to decrement the table index.
     *
     * @return the task
     */
    public Task decrementTask() {
        return new RunTask(this::decrement, this, false).withName("Decrement Index");
    }

    /**
     * Set the table index.
     *
     * @param index the index to set
     */
    public void set(int index) {
        if (index < 0 || index >= tableValues.length) {
            throw new IllegalArgumentException("Index out of bounds");
        }
        this.index = index;
    }

    /**
     * Get the applied value from the index.
     *
     * @return the table value
     */
    public double get() {
        return tableValues[index];
    }

    /**
     * Optionally update telemetry with the current value and index.
     */
    @Override
    protected void periodic() {
        opMode.telemetry.add(
                "%: % <font color='gray'>(%/%)</font>",
                NAME != null ? "Idx-" + NAME : "Index Table",
                tableValues[index],
                index + 1,
                tableValues.length
        );
    }
}
