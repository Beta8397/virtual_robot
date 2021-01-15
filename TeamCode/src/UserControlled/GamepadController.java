package UserControlled;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
    Author: Ethan Fisher
    Date: 10/23/2020

    Makes it easier to use the gamepads
    For example, if you want to only allow the user to press a button
    and not hold it, this object will allow you to do so
 */
public class GamepadController {

    private final Gamepad gamepad;

    private static final double TRIGGER_THRESHOLD = 0.1;

    public Action OnAPressed, OnBPressed, OnXPressed, OnYPressed, OnRightBumperPressed,
            OnLeftBumperPressed, OnRightTriggerPressed, OnLeftTriggerPressed, OnDPadUpPressed,
            OnDPadRightPressed, OnDPadDownPressed, OnDPadLeftPressed, OnStartPressed;

    public MovementAction OnLeftStickMove, OnRightStickMove;

    public boolean aPressed = false, aHeld = false, bPressed = false, bHeld = false,
            xPressed = false, xHeld = false, yPressed = false, yHeld = false,
            rightBumperPressed = false, rightBumperHeld = false, leftBumperPressed = false,
            leftBumperHeld = false, rightTriggerPressed = false, rightTriggerHeld = false,
            leftTriggerPressed = false, leftTriggerHeld = false, dpadUpPressed = false,
            dpadUpHeld = false, dpadRightPressed = false, dpadRightHeld = false,
            dpadDownPressed = false, dpadDownHeld = false, dpadLeftPressed = false,
            dpadLeftHeld = false, startPressed = false, startHeld = false;

    public GamepadController(Gamepad gamepad) { this.gamepad = gamepad; }

    public void update() {

        if (OnLeftStickMove != null)
            OnLeftStickMove.Invoke(gamepad.left_stick_x, -gamepad.left_stick_y);
        if (OnRightStickMove != null)
            OnRightStickMove.Invoke(gamepad.right_stick_x, -gamepad.right_stick_y);

        // update a
        boolean prevAHeld = aHeld;
        aHeld = gamepad.a;
        if (prevAHeld) aPressed = false;
        if (!prevAHeld && aHeld) aPressed = true;
        if (aPressed && OnAPressed != null) OnAPressed.OnAction();

        // update b
        boolean prevBHeld = bHeld;
        bHeld = gamepad.b;
        if (prevBHeld) bPressed = false;
        if (!prevBHeld && bHeld) bPressed = true;
        if (bPressed && OnBPressed != null) OnBPressed.OnAction();

        // update x
        boolean prevXHeld = xHeld;
        xHeld = gamepad.x;
        if (prevXHeld) xPressed = false;
        if (!prevXHeld && xHeld) xPressed = true;
        if (xPressed && OnXPressed != null) OnXPressed.OnAction();

        // update y
        boolean prevYHeld = yHeld;
        yHeld = gamepad.y;
        if (prevYHeld) yPressed = false;
        if (!prevYHeld && yHeld) yPressed = true;
        if (yPressed && OnYPressed != null) OnYPressed.OnAction();

        // update right bumper
        boolean prevRightBumperHeld = rightBumperHeld;
        rightBumperHeld = gamepad.right_bumper;
        if (prevRightBumperHeld) rightBumperPressed = false;
        if (!prevRightBumperHeld && rightBumperHeld) rightBumperPressed = true;
        if (rightBumperPressed && OnRightBumperPressed != null) OnRightBumperPressed.OnAction();

        // update left bumper
        boolean prevLeftBumperHeld = leftBumperHeld;
        leftBumperHeld = gamepad.left_bumper;
        if (prevLeftBumperHeld) leftBumperPressed = false;
        if (!prevLeftBumperHeld && leftBumperHeld) leftBumperPressed = true;
        if (leftBumperPressed && OnLeftBumperPressed != null) OnLeftBumperPressed.OnAction();

        //update right trigger
        boolean prevRightTriggerHeld = rightTriggerHeld;
        rightTriggerHeld = isTriggerHeld(gamepad.right_trigger);
        if (prevRightTriggerHeld) rightTriggerPressed = false;
        if (!prevRightTriggerHeld && rightTriggerHeld) rightTriggerPressed = true;
        if (rightTriggerPressed && OnRightTriggerPressed != null) OnRightTriggerPressed.OnAction();

        //update left trigger
        boolean prevLeftTriggerHeld = leftTriggerHeld;
        leftTriggerHeld = isTriggerHeld(gamepad.left_trigger);
        if (prevLeftTriggerHeld) leftTriggerPressed = false;
        if (!prevLeftTriggerHeld && leftTriggerHeld) leftTriggerPressed = true;
        if (leftTriggerPressed && OnLeftTriggerPressed != null) OnLeftTriggerPressed.OnAction();

        // update d pad up
        boolean prevDPadUpHeld = dpadUpHeld;
        dpadUpHeld = gamepad.dpad_up;
        if (prevDPadUpHeld) dpadUpPressed = false;
        if (!prevDPadUpHeld && dpadUpHeld) dpadUpPressed = true;
        if (dpadUpPressed && OnDPadUpPressed != null) OnDPadUpPressed.OnAction();

        // update d pad right
        boolean prevDPadRightHeld = dpadRightHeld;
        dpadRightHeld = gamepad.dpad_right;
        if (prevDPadRightHeld) dpadRightPressed = false;
        if (!prevDPadRightHeld && dpadRightHeld) dpadRightPressed = true;
        if (dpadRightPressed && OnDPadRightPressed != null) OnDPadRightPressed.OnAction();

        // update d pad down
        boolean prevDPadDownHeld = dpadDownHeld;
        dpadDownHeld = gamepad.dpad_down;
        if (prevDPadDownHeld) dpadDownPressed = false;
        if (!prevDPadDownHeld && dpadDownHeld) dpadDownPressed = true;
        if (dpadDownPressed && OnDPadDownPressed != null) OnDPadDownPressed.OnAction();

        // update d pad left
        boolean prevDPadLeftHeld = dpadLeftHeld;
        dpadLeftHeld = gamepad.dpad_left;
        if (prevDPadLeftHeld) dpadLeftPressed = false;
        if (!prevDPadLeftHeld && dpadLeftHeld) dpadLeftPressed = true;
        if (dpadLeftPressed && OnDPadLeftPressed != null) OnDPadLeftPressed.OnAction();

        // update start button
        boolean prevStartHeld = startHeld;
        startHeld = gamepad.start;
        if (prevStartHeld) startPressed = false;
        if (!prevStartHeld && startHeld) startPressed = true;
        if (startPressed && OnStartPressed != null) OnStartPressed.OnAction();
    }

    public double calculateLeftStickAngle() {
        double angle = Math.toDegrees(Math.atan2(gamepad.left_stick_x, -gamepad.left_stick_y));
        if (Double.isNaN(angle)) return 0;
        return angle;
    }

    public double calculateRightStickAngle() {
        double angle = Math.toDegrees(Math.atan2(gamepad.right_stick_x, -gamepad.right_stick_y));
        if (Double.isNaN(angle)) return 0;
        return angle;
    }

    public double calculateLeftStickMagnitude() {
        return Math.sqrt(Math.pow(gamepad.left_stick_x, 2) + Math.pow(gamepad.left_stick_y, 2));
    }

    public double calculateRightStickMagnitude() {
        return Math.sqrt(Math.pow(gamepad.right_stick_x, 2) + Math.pow(gamepad.right_stick_y, 2));
    }

    private boolean isTriggerHeld(double trigger) {
        return trigger > TRIGGER_THRESHOLD;
    }
}
