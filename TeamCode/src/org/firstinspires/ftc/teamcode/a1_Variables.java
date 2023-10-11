package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Variables (see chapter 2 in Learn Java for FTC)
 * It's useful to store values in our program so we can do math with them, use
 * them later, or compare them to other things in the future. To do that, we
 * use things called variables (because they can change or vary) which give
 * those values a name. To use a variable we have to pick declare it.
 *
 * //This is a variable declaration
 * int variableName;
 *
 * //You can also assign it a value when you declare it
 * int variableName = 101;
 *
 * //But when you use it later, just include the name (not the type)
 * variableName = 5;
 *
 * The common types we'll use are
 *      int - whole numbers (no decimal, eg 5)
 *      double - number with decimals (eg 5.115)
 *      boolean - can either true or false
 *      String - used to store text ( a string of characters)
 *
 * Do these experiments using any Robot and running the Lesson 1: Variables
 * 1. Change the variable name to be your name and then add a telemetry.addData
 *    call that will run after we hit the init button to print
 *      My name is: [name]
 *    Don't forget the telemetry.update()
 * 2. Inside the while loop. Add telemetry.addData calls to write the variables
 *    speed, count, and teamPropDetected to the Driver Hub.
 * 3. Declare a new int variable called test with the other variables and set its value to 5.5.
 *     Add a telemetry.addData with the others to print its value too.
 * 4. Notice how that value isn't the same as when you declared it. Change its
 *    type to a double and try it again.
 * 5. Add this snippet of code to the while loop, before the telemetry calls you added.
 *      if(gamepad1.a){
 *              teamPropDetected = true;
 *       }
 *       else{
 *           teamPropDetected = false;
 *       }
 *     You should see this value toggle between true and false when you press the a button.
 *
 */
@TeleOp(name = "Lesson 1: Variables")
public class a1_Variables extends LinearOpMode {
    //Declare variables here
    //#3 goes here
    String name = "Put your name here";
    double speed = 1;
    int count = 0;
    boolean teamPropDetected = false;

    public void runOpMode() throws InterruptedException {
        //Code that runs when we hit init.
        //#1 goes here

        waitForStart();

        while(opModeIsActive()){
            //#2 goes here
        }

    }
}
