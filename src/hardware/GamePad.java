package hardware;

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
}
