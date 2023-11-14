package org.firstinspires.ftc.teamcode.example.examplerobot.components;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.common.BunyipsComponent;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;

/**
 * Example code for a drive system using the BunyipsOpMode ecosystem.
 */
// Extend the `BunyipsComponent` class when making a new component, as shown below.
public class ExampleDrive extends BunyipsComponent {
    // You will need to store any hardware you wish to control in your component as another instance
    // (similar to the config class file)
    // For example, if you have a motor called `leftMotor`, you will need to declare it like this:
    private final DcMotor leftMotor;
    // ! Note that these variables are private and not public, as the component should handle full
    // control and access permissions to the hardware it is controlling.
    private final DcMotor rightMotor;

    // You will also declare any local variables that you wish to use in your component.
    private double motorPower;

    // This constructor will be automatically placed, and this allows us to access the BunyipsOpMode
    // instance from anywhere inside this example component. To access this instance, we call
    // a built-in function called getOpMode(), which will return the BunyipsOpMode instance.
    public ExampleDrive(@NonNull BunyipsOpMode opMode, DcMotor left, DcMotor right) {
        // Required to delegate the BunyipsOpMode instance to the superclass.
        super(opMode);

        // Pass additional devices that you wish to control in your component within the
        // constructor, such as motors for a drive system, as shown above and below.
        // Assigns constructor instances to the private variables declared above, allowing us to
        // use them in the component.
        leftMotor = left;
        rightMotor = right;
    }

    // This covers all required functions that need to be implemented from the BunyipsComponent,
    // and below you can add any additional functions that you wish to use in your component.
    // This may include methods to controlling these motors, such as a drive function.

    // This example will cover a simple drive function, which will take in a power and set the
    // local variable to represent this power. (This is also called a setter)
    public void run(double power) {
        // Set the power of the motors to the given power.
        motorPower = power;
    }

    public void stop() {
        // Stop the motors by setting power to zero
        motorPower = 0;
    }

    public boolean isAtPosition(double target) {
        return leftMotor.getCurrentPosition() > target;
    }

    // ALL components should strive to include an update() method, which will be called every
    // hardware cycle in the activeLoop component. This is where the actual hardware control should
    // strive to occur.
    public void update() {
        // Reflect local variable changes to the actual motors
        leftMotor.setPower(motorPower);
        rightMotor.setPower(-motorPower);
    }

}
