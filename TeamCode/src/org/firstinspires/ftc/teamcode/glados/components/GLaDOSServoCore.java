package org.firstinspires.ftc.teamcode.glados.components;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.common.BunyipsComponent;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;

/**
 * Controller for the Pixel Intake servo mechanism for GLaDOS.
 *
 * @author Lucas Bubner, 2023
 */
public class GLaDOSServoCore extends BunyipsComponent {
    private static final double LEFT_SERVO_CLOSED_POSITION = 0.0;
    private static final double LEFT_SERVO_OPEN_POSITION = 0.5;
    private static final double RIGHT_SERVO_CLOSED_POSITION = 1.0;
    private static final double RIGHT_SERVO_OPEN_POSITION = 0.5;
    private final Servo leftServo;
    private final Servo rightServo;
    private double leftServoPosition;
    private double rightServoPosition;

    public GLaDOSServoCore(@NonNull BunyipsOpMode opMode, Servo leftServo, Servo rightServo) {
        super(opMode);
        this.leftServo = leftServo;
        this.rightServo = rightServo;

        // Always close on init
        leftServoPosition = LEFT_SERVO_CLOSED_POSITION;
        rightServoPosition = RIGHT_SERVO_CLOSED_POSITION;
        update();
    }

    public void toggleServo(ServoSide servo) {
        if (servo == ServoSide.LEFT) {
            leftServoPosition = (leftServoPosition == LEFT_SERVO_OPEN_POSITION) ? LEFT_SERVO_CLOSED_POSITION : LEFT_SERVO_OPEN_POSITION;
            return;
        }
        rightServoPosition = (rightServoPosition == RIGHT_SERVO_OPEN_POSITION) ? RIGHT_SERVO_CLOSED_POSITION : RIGHT_SERVO_OPEN_POSITION;
    }

    public void openServo(ServoSide servo) {
        if (servo == ServoSide.LEFT) {
            leftServoPosition = LEFT_SERVO_OPEN_POSITION;
            return;
        }
        rightServoPosition = RIGHT_SERVO_OPEN_POSITION;
    }

    public void closeServo(ServoSide servo) {
        if (servo == ServoSide.LEFT) {
            leftServoPosition = LEFT_SERVO_CLOSED_POSITION;
            return;
        }
        rightServoPosition = RIGHT_SERVO_CLOSED_POSITION;
    }

    /**
     * Push stateful changes to the servos.
     */
    public void update() {
        leftServo.setPosition(leftServoPosition);
        rightServo.setPosition(rightServoPosition);
        getOpMode().addTelemetry("Servos: L_% R_%", leftServo.getPosition() == LEFT_SERVO_OPEN_POSITION ? "OPEN" : "CLOSE", rightServo.getPosition() == RIGHT_SERVO_OPEN_POSITION ? "OPEN" : "CLOSE");
    }

    public enum ServoSide {
        LEFT,
        RIGHT
    }
}
