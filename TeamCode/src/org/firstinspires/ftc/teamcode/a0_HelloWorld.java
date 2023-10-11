package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 *  Telemetry is a basic function of the FTC API that we use all the time to
 *  send debugging data to the Driver Hub.
 *
 *  The telemetry object is already a part of our code as long as the class
 *  we're in extends OpMode or LinearOpMode (as this one does).
 *
 *  There are a ton of telemetry functions available to us, such as
 *  telemetry.addData(String caption, Object item);
 *      -Displays like "caption: item" on the Driver Hub.
 *  telemetry.addLine(String lineCaption);
 *      -Just displays the text entered. This is useful for adding headings,
 *       general statements, or custom formatted text.
 *  telemetry.update();
 *      -Sends the queued transmissions to the Driver Hub.
 *  telemetry.setAutoClear(Boolean autoClear);
 *      -Allows you to change whether old data should stay on the display when
 *       you call update.
 *  telemetry.clear() and telemetry.clearAll()
 *      -Lets you manually clear telemetry when AutoClear is disabled.
 *
 *  Do these experiments using any Robot and running the Lesson 0: Hello World OpMode
 *  1. Use telemetry.addData or telemetry.addLine to write the text "Hello World".
 *     Make sure you add a line below it to call telemetry.update()
 *  2. Use another telemetry.addLine to make the full text:
 *      Hello World!
 *      My name is [your name here].
 *  3. Add another telemetry call after waitForStart() so it will print
 *     "The program is running now" after we hit Start. Make sure to call
 *     telemetry.update() again.
 *  4. Notice how that clears all the text we had written. Add the statement
 *     telemetry.setAutoClear(false) before any of your telemetry lines. Now
 *     the text should stay around until we manually clear it.
 *  5. Let's try clearing telemetry now. Add this bit of code to the while loop:
 *            if(gamepad1.a){
 *                 telemetry.clear();
 *                 telemetry.update();
 *             }
 *
 *      When you press the A button (while the program is running) it should clear all text.
 */
@TeleOp(name = "Lesson 0: Hello World")
public class a0_HelloWorld extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        //Add your telemetry for #1, #2, & #4 here.




        // This line stops the program here until you hit the start button.
        waitForStart();
        //#3 goes here
        //This is where our code runs after you hit the Start button.



        while(opModeIsActive()){
            //This is our main loop.
            //#5 goes here.

        }
    }
}
