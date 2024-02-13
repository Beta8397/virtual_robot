package org.murraybridgebunyips.bunyipslib.example.examplerobot.components;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

// See ExampleDrive before looking at this file.
public class ExampleLift extends BunyipsSubsystem {
    // Run similar initialisation to ExampleDrive. This will be the same for all components.
    private final DcMotor liftMotor;

    // This example will look at making an index-based lift based on motor encoder position.
    // Array to hold lift position values, and a pointer to the current position.
    private final int[] liftPositions = {0, 100, 200, 300, 400, 500};
    private int liftPositionPointer;

    public ExampleLift(DcMotor liftMotor) {
        this.liftMotor = liftMotor;

        // In the constructor you are also able to set any other variables that you wish to use in
        // your component, such as the lift position pointer.
        liftPositionPointer = 0;
    }

    public void liftUp() {
        // Check if the pointer is exceeding the length of the array
        if (liftPositionPointer >= liftPositions.length) {
            // If it is, set the pointer to the last index of the array
            liftPositionPointer = liftPositions.length - 1;
            return;
        }
        // Increment the pointer
        liftPositionPointer++;
    }

    public boolean isBusy() {
        return liftMotor.isBusy();
    }

    public void liftDown() {
        // Do the opposite of liftUp()
        if (liftPositionPointer <= 0) {
            liftPositionPointer = 0;
            return;
        }
        liftPositionPointer--;
    }

    public void update() {
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
        opMode.addTelemetry("Lift Position: " + position);
    }
}
