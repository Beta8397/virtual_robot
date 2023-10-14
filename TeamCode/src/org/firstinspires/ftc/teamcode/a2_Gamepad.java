package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * The two Gamepads are our interface to the robot. They provide several types
 * of data that we can use control motor speeds and toggle robot states.
 *
 * The gamepads can return two types of values - doubles and booleans.
 *
 * Analog Values:
 *      - Joysticks have two axes (X - left/right, and Y - forward/backward)
 *        and return a value between -1 and 1. They're great for controlling
 *        motor speeds and robot movement.
 *      - Triggers return a value between 0 (not pressed) and 1 (fully pressed).
 *        They can run variable speed intakes with separate buttons for intake
 *        and outtake, or you can combine the two inputs to control things like
 *        robot orientation.
 *
 * Boolean Values:
 *      - The A/B/X/Y buttons, bumpers, D-Pad, and joystick buttons all return
 *        booleans. These buttons work well for fixed speed intakes, running
 *        motors to specific positions, and changing robot modes.
 *
 *  Exercises:
 *  1. Use telemetry.addData to output the X and Y axes. One is already done for
 *     you. Don't forget to add a telemetry.update();
 *  2. We can save gamepad values to variables too. A variable already exists
 *     for the X axis, add another one for Y axis and assign them both inside
 *     of the while loop. Update the telemetry calls to use those variables.
 *  3. Notice how the Y axis value is negative. Add a negative sign (-) to that
 *     to invert the value and make it positive when the stick is forward.
 *  4. There is a variable called "abutton" declared in the program. Set the
 *     value to gamepad.a in the while loop and add its value to telemetry.
 *  5. Add this section of code to the beginning of the while loop and then
 *     add toggleValue to telemetry.
 *      if(abutton && !gamepad1.a){
 *             toggleValue = !toggleValue;
 *      }
 *      This value should switch between true and false you press the a button.
 *
 */

@TeleOp(name = "Lesson 2: Gamepad")
public class a2_Gamepad extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        //Declare variables here
        //#2 goes here
        double leftX;
        boolean abutton = false;
        boolean toggleValue = false;

        waitForStart();

        while(opModeIsActive()){
            //#1 goes here
            telemetry.addData("Left-Y", gamepad1.left_stick_y);


        }

    }
}
