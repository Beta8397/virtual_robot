package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * The numbers we get from sensors and gamepads are useful, but they often need
 * some adjustments to do what we need (like the negative joystick y-axis that
 * we need to invert). Here are the basic math operators available to us for
 * manipulating numbers:
 *      =   This is the assignment operator, we use to set variable values
 *      +   This is the addition operator, it's used to add numbers together
 *      -   This is the subtraction and negative operator, it can subtract
 *          numbers and make them negative (remember that negating a negative
 *          number yields a positive number).
 *      *   This is the multiplication operator
 *      /   This is the division operator. Remember that dividing by zero is
 *          impossible, it will trigger a runtime error and crash your program
 *          if you do it.
 *      %   This is the modulo - or mod - operator. It returns the remainder of
 *          a division operation (eg. 5 % 2 returns 1)
 *
 * Equations that you use follow standard order of operations ( Parentheses ->
 * Multiplication/Division -> Addition/Subtraction) but make sure to use
 * parentheses wisely to get the results you want.
 *
 * Let's try some these out with these exercises:
 * 1. Declare a double variable called rightStickY. Inside the while loop, assign
 *    that variable to gamepad1.right_stick_y and output the variable's value
 *    using telemetry.
 * 2. Add a negative (-) sign where you are assigning rightStickY to make it
 *    positive when the stick is forward.
 * 3. In the same assignment, use the division operator (/) to divide the
 *    joystick's value by 2, so pushing all the way forward yields .5
 * 4. On the next line, assign rightStickY (again) to equal itself multiplied
 *    by 2. Now pushing forward all the way yields 1 again.
 * 5. Declare a new int variable called aButtonCount and initialize it to 0.
 *    Add it to telemetry. There is a pre-written if statement that will execute
 *    the code inside it once each time the A button is pressed. Add a line in
 *    the if block to assign aButtonCount to itself plus 1.
 */
@TeleOp(name = "Lesson 3: Basic Math")
public class a3_BasicMath extends LinearOpMode {
    //Variables go here
    //#1 goes here.
    boolean abutton = false;


    public void runOpMode() throws InterruptedException {

        waitForStart();

        while(opModeIsActive()){
            //Variable assignment goes here


            if(aButtonPressed()){
                //#5 goes here
            }

            //Telemetry goes here

            telemetry.update();

        }

    }

    /*
        Returns true if the A button was just pressed.
     */
    boolean aButtonPressed(){
        boolean returnValue = false;

        if(!abutton && gamepad1.a){
            returnValue = true;
        }
        abutton = gamepad1.a;
        return returnValue;
    }
}
