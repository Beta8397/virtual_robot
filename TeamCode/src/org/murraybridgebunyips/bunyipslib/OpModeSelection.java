package org.murraybridgebunyips.bunyipslib;


/**
 * OpMode selection options for AutonomousBunyipsOpMode.
 *
 * @author Lucas Bubner, 2023
 */
public class OpModeSelection {
    private final String name;
    private final Object obj;
    private Controller assignedButton;

    public OpModeSelection(Object obj) {
        if (obj instanceof Enum) {
            name = ((Enum<?>) obj).name();
        } else {
            name = obj.toString();
        }
        this.obj = obj;
    }

    public Controller getAssignedButton() {
        return assignedButton;
    }

    public void setAssignedButton(Controller assignedButton) {
        this.assignedButton = assignedButton;
    }

    public String getName() {
        return name;
    }

    public Object getObj() {
        return obj;
    }

    @Override
    public boolean equals(Object o) {
        return name.equals(o) || obj == o || o == this;
    }
}
