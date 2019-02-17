package virtual_robot.hardware;

import com.studiohartman.jamepad.ControllerManager;
import com.studiohartman.jamepad.ControllerState;

/**
 * Represents the GamePad.
 *
 * Note: the fields in the class are all public, but they should not be changed from the OpMode code.
 */
public class GamePad {

    public volatile boolean x = false;
    public volatile boolean y = false;
    public volatile boolean a = false;
    public volatile boolean b = false;
    public volatile float left_stick_x  = 0;
    public volatile float left_stick_y = 0;
    public volatile float right_stick_x = 0;
    public volatile float right_stick_y = 0;
    public volatile boolean dpad_up = false;
    public volatile boolean dpad_down = false;
    public volatile boolean dpad_left = false;
    public volatile boolean dpad_right = false;
    public volatile boolean back = false;
    public volatile boolean guide = false;
    public volatile boolean start = false;
    public volatile boolean left_bumper = false;
    public volatile boolean right_bumper = false;
    public volatile boolean left_stick_button = false;
    public volatile boolean right_stick_button = false;
    public volatile float left_trigger = 0;
    public volatile float right_trigger = 0;

    private ControllerManager controllers = new ControllerManager();

    public GamePad(){
        controllers.initSDLGamepad();
        update();
    }

    public void update(){
        ControllerState state = controllers.getState(0);
        x = state.x;
        y = state.y;
        a = state.a;
        b = state.b;
        left_stick_x = state.leftStickX;
        left_stick_y = -state.leftStickY;
        right_stick_x = state.rightStickX;
        right_stick_y = -state.rightStickY;
        dpad_up = state.dpadUp;
        dpad_down = state.dpadDown;
        dpad_left = state.dpadLeft;
        dpad_right = state.dpadRight;
        back = state.back;
        guide = state.guide;
        start = state.start;
        left_bumper = state.lb;
        right_bumper = state.rb;
        left_stick_button = state.leftStickClick;
        right_stick_button = state.rightStickClick;
        left_trigger = state.leftTrigger;
        right_trigger = state.rightTrigger;
    }

    public void release(){
        controllers.quitSDLGamepad();
    }

}
