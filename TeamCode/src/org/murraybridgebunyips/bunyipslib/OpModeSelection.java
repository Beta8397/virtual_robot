package org.murraybridgebunyips.bunyipslib;


import androidx.annotation.NonNull;

/**
 * OpMode selection options for AutonomousBunyipsOpMode.
 *
 * @author Lucas Bubner, 2023
 */
public class OpModeSelection {
    private final String name;
    private final Object obj;
    private Controls assignedButton;

    /**
     * Wrap an object with a name for selection.
     *
     * @param obj the object to wrap
     */
    public OpModeSelection(Object obj) {
        if (obj instanceof Enum) {
            name = ((Enum<?>) obj).name();
        } else {
            name = obj.toString();
        }
        this.obj = obj;
    }

    public Controls getAssignedButton() {
        return assignedButton;
    }

    public void setAssignedButton(Controls assignedButton) {
        this.assignedButton = assignedButton;
    }

    public Object getObj() {
        return obj;
    }

    @NonNull
    @Override
    public String toString() {
        return name;
    }

    @SuppressWarnings("EqualsWhichDoesntCheckParameterClass")
    @Override
    public boolean equals(Object o) {
        return obj == o || o == this || obj.toString().equals(o.toString()) || name.equals(o.toString());
    }
}
