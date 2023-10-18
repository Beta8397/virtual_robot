package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * In our programs, we need to do different things based on gamepad input, sensor
 * values, timers, and robot states. To do this, we can use conditions with
 * if-else if-else blocks.
 *
 * First lets talk about if statements. An if-else if-else block looks like this:
 *
 * if(condition){
 *     //run the code here if the condition is true
 *     // You can use an if statement by itself - it works just like having an
 *     // empty else.
 * }
 * else if(condition 2){
 *     //run the code here if condition 1 is false and condition 2 is true
 *     //you can include as many else if blocks as you want, but they must follow
 *     //an if statement or an else if statement
 * }
 * else{
 *     //run this code if conditions 1 & 2 are both false
 *     //this can follow an if or an else if, but it must be the last statement
 *     // in the block.
 * }
 *
 * The conditions in an if or else if statement must be boolean (true or false)
 * values. We can use boolean values (like gamepad buttons) directly, or we can
 * use comparison operator operators to write a boolean statement.
 *
 * These are the basic comparison operators and examples for using them:
 *      == this is the equality operator, it returns true if the things compared
 *         are equal.
 *         10 == 5 //this would return false
 *      >  this is the greater than operator, it returns true if the left operand
 *         is greater than the right operand.
 *         10 > 5 //this would return true
 *      <  this is the greater than operator, it returns true if the left operand
 *         is less than the right operand.
 *         10 < 5 //this would return false
 *      >= this is the greater than or equal to operator, it returns true if the
 *         left operand is greater than or equal to the right operand
 *      <= this is the less than or equal to operator, it returns true if the
 *         left operand is less than or equal to operand
 *      && this is the and operator. It returns true if the left and right
 *         operands are both true
 *         (true) || (false) // this would return false
 *      || this is the or operator. It returns true if either left or right
 *         operand is true.
 *         (true) || (false) // this would return true
 *      !  this is the not operator, it returns the opposite of the operand that
 *         follows it.
 *         !(true) // this would return false
 *      != this is the not equals operator, it returns true if the left and right
 *         operands are not equal
 *         1 != 5 // this would return true
 *
 * Here's an example of how we could use and if-else if-else block with comparisons:
 *
 * if(gamepad1.left_stick_y > 0){
 *     telemetry.addLine("Left stick is positive");
 * }
 * else if(gamepad1.left_stick_y < 0){
 *     telemetry.addLine("Left stick is negative");
 * }
 * else{
 *     telemetry.addLine("Left Stick is zero");
 * }
 *
 * If statements can get pretty complicated. Here's an example of a nested if
 * statement:
 *
 * if(gamepad1.a || gamepad1.b){
 *     if(gamepad1.a){
 *         telemetry.addLine("A button is pressed.");
 *     }
 *     if(gamepad1.b){
 *         telemetry.addLine("B button is pressed.");
 *     }
 * }
 * else{
 *     telemetry.addLine("The A and B buttons are not pressed.");
 * }
 *
 * Let's try using it.
 * 1. Add if statements for each of the A and B buttons that will
 *    write something like "The A button is pressed" to telemetry.
 * 2. Add an if statment that uses the or operator to write "The X or Y button
 *    is pressed" if the X or Y buttons are pressed.
 * 3. Use an if-else if-else block to compare gamepad1.right_stick_x and
 *    gamepad1.right_stick_y. It should write the following to telemetry:
 *    "Y Greater than X" when the Y axis is greater than the X axis
 *    "X Greater than Y" when the X axis is greater than the Y axis
 *    "Y axis equal to X axis" when they are the same.
 * 4. Use ane if-else block to write "Trigger is pressed" when
 *    gamepad2.right_trigger > .25 and "Trigger not pressed" otherwise.
 * 5. The A buttons counter from the last program has already been copied
 *    to this program. Use an if-else if-else block to write the following to
 *    telemetry:
 *      "You haven't pressed the button yet" when aButtonCount == 0
 *      "You've pressed the button once" when aButtonCount == 1
 *      "You've pressed the button a couple times" when aButtonCount == 2
 *      "You've pressed the button a few times" when 2< aButtonCount < 5
 *      "You've pressed the button several times" when aButtonCount >= 5
 *      "You can stop pressing the button now" when aButtonCount > 20
 *  6. *Optional* You can do a modified version of 5 where the program
 *      tries to convince the user to stop pressing the A button, with
 *      increasing frantic dialogs the more the user presses it.
 *
 */
@TeleOp(name = "Lesson 4: Conditions")
public class a4_Conditions extends LinearOpMode {
    boolean abutton = false;
    int aButtonCount = 0;

    public void runOpMode() throws InterruptedException {
        waitForStart();

        while(opModeIsActive()){
            if(aButtonPressed()) {
                aButtonCount++;
            }

            //All of your code goes below here




            telemetry.update();
        }
    }

    boolean aButtonPressed(){
        boolean returnValue = false;

        if(!abutton && gamepad1.a){
            returnValue = true;
        }
        abutton = gamepad1.a;
        return returnValue;
    }
}
