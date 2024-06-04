package org.murraybridgebunyips.bunyipslib.example.examplerobot.components;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * Example code for a lift component, see ExampleDrive for more information.
 */
public class ExampleLift extends BunyipsSubsystem {
    // This example will look at making an index-based lift based on motor encoder position.
    // Array to hold lift position values, and a pointer to the current position.
    private final int[] liftPositions = {0, 100, 200, 300, 400, 500};
    // Run similar initialisation to ExampleDrive. This will be the same for all components.
    private DcMotor liftMotor;
    private int liftPositionPointer;

    /**
     * @param liftMotor the lift motor to control
     */
    public ExampleLift(DcMotor liftMotor) {
        if (!assertParamsNotNull(liftMotor)) return;
        this.liftMotor = liftMotor;

        // In the constructor you are also able to set any other variables that you wish to use in
        // your component, such as the lift position pointer.
        liftPositionPointer = 0;
    }

    /**
     * Move the lift up one position.
     *
     * @return this
     */
    public ExampleLift liftUp() {
        // Check if the pointer is exceeding the length of the array
        if (liftPositionPointer >= liftPositions.length) {
            // If it is, set the pointer to the last index of the array
            liftPositionPointer = liftPositions.length - 1;
            return this;
        }
        // Increment the pointer
        liftPositionPointer++;
        // We also use a builder pattern to allow for chaining such as lift.liftUp().update();
        // This is optional and your methods can have a void return type if you don't want to chain.
        return this;
    }

    public boolean isBusy() {
        return liftMotor.isBusy();
    }

    /**
     * Move the lift down one position.
     *
     * @return this
     */
    public ExampleLift liftDown() {
        // Do the opposite of liftUp()
        if (liftPositionPointer <= 0) {
            liftPositionPointer = 0;
            return this;
        }
        liftPositionPointer--;
        return this;
    }

    @Override
    protected void periodic() {
        // Use the array and pointer to determine what position the motor should be set to.
        int position = liftPositions[liftPositionPointer];

        // Set the motor to the position
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Run the motor
        liftMotor.setPower(0.2);

        // Can update telemetry functions too
        // The modified telemetry function takes in a value to show on the Driver Station, and
        // whether or not to keep it on the screen upon the next activeLoop.
        opMode.telemetry.add("Lift Position: " + position);
    }
}
