package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.Controls.getAxes;
import static org.murraybridgebunyips.bunyipslib.Controls.getButtons;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.function.Function;
import java.util.function.Predicate;

/**
 * A wrapper around a {@link Gamepad} object that provides a {@link Controls} interface and custom input calculations.
 * These gamepad objects are used natively in BunyipsOpMode, and are the drop-in replacements for gamepad1 and gamepad2.
 *
 * @author Lucas Bubner, 2024
 * @see BunyipsOpMode
 */
public class Controller extends Gamepad {
    /**
     * A function that returns the input value as-is.
     */
    public static final Function<Float, Float> LINEAR = x -> x;
    /**
     * A function that squares the input value and keeps the sign.
     */
    public static final Function<Float, Float> SQUARE = x -> Math.copySign(x * x, x);
    /**
     * A function that cubes the input value and keeps the sign.
     */
    public static final Function<Float, Float> CUBE = x -> Math.copySign(x * x * x, x);
    /**
     * A function that negates the input value.
     */
    public static final Function<Float, Float> NEGATE = x -> -x;
    /**
     * The SDK gamepad that this Controller wraps and takes input from.
     * This is public for advanced users who need to access the raw gamepad values, or need to access hardware
     * related gamepad methods that are not available in the Controller class which copies state.
     */
    public final Gamepad sdk;
    private final HashMap<Controls, Predicate<Boolean>> buttons = new HashMap<>();
    private final HashMap<Controls.Analog, Function<Float, Float>> axes = new HashMap<>();
    private final HashMap<Controls, Boolean> debounces = new HashMap<>();
    /**
     * Shorthand for left_stick_x
     */
    public volatile float lsx;
    /**
     * Shorthand for left_stick_y
     */
    public volatile float lsy;
    /**
     * Shorthand for right_stick_x
     */
    public volatile float rsx;
    /**
     * Shorthand for right_stick_y
     */
    public volatile float rsy;
    /**
     * Shorthand for left_trigger
     */
    public volatile float lt;
    /**
     * Shorthand for right_trigger
     */
    public volatile float rt;
    /**
     * Shorthand for left_bumper
     */
    public volatile boolean lb;
    /**
     * Shorthand for right_bumper
     */
    public volatile boolean rb;
    /**
     * Shorthand for dpad_up
     */
    public volatile boolean du;
    /**
     * Shorthand for dpad_down
     */
    public volatile boolean dd;
    /**
     * Shorthand for dpad_left
     */
    public volatile boolean dl;
    /**
     * Shorthand for dpad_right
     */
    public volatile boolean dr;
    /**
     * Shorthand for left_stick_button
     */
    public volatile boolean lsb;
    /**
     * Shorthand for right_stick_button
     */
    public volatile boolean rsb;

    /**
     * Create a new Controller to manage.
     *
     * @param gamepad The Gamepad to wrap (gamepad1, gamepad2)
     */
    public Controller(Gamepad gamepad) {
        sdk = gamepad;
    }

    private void parseUnmanagedControllerBuffer() {
        ByteBuffer byteBuffer = ByteBuffer.wrap(sdk.toByteArray());

        int buttons;
        byte version = byteBuffer.get();

        if (version >= 1) {
            id = byteBuffer.getInt();
            timestamp = byteBuffer.getLong();
            // Skip over buffers we don't care about
            for (int i = 0; i < 6; i++) {
                byteBuffer.getFloat();
            }

            buttons = byteBuffer.getInt();
            touchpad_finger_1 = (buttons & 0x20000) != 0;
            touchpad_finger_2 = (buttons & 0x10000) != 0;
            touchpad = (buttons & 0x08000) != 0;
            guide = (buttons & 0x00010) != 0;
        }

        if (version >= 2) {
            user = byteBuffer.get();
        }

        if (version >= 3) {
            type = Type.values()[byteBuffer.get()];
        }

        if (version >= 4) {
            byte v4TypeValue = byteBuffer.get();
            if (v4TypeValue < Type.values().length) {
                type = Type.values()[v4TypeValue];
            }
        }

        if (version >= 5) {
            touchpad_finger_1_x = byteBuffer.getFloat();
            touchpad_finger_1_y = byteBuffer.getFloat();
            touchpad_finger_2_x = byteBuffer.getFloat();
            touchpad_finger_2_y = byteBuffer.getFloat();
        }
    }

    /**
     * Update the public fields of this Controller with the values from the wrapped Gamepad, performing calculations
     * on the inputs as specified by the user.
     * This method must be called in a loop of the OpMode to update the Controller's values. This is automatically
     * called in BunyipsOpMode on another thread.
     */
    public void update() {
        parseUnmanagedControllerBuffer();

        // Recalculate all custom inputs
        left_stick_x = get(Controls.Analog.LEFT_STICK_X);
        left_stick_y = get(Controls.Analog.LEFT_STICK_Y);
        right_stick_x = get(Controls.Analog.RIGHT_STICK_X);
        right_stick_y = get(Controls.Analog.RIGHT_STICK_Y);
        left_trigger = get(Controls.Analog.LEFT_TRIGGER);
        right_trigger = get(Controls.Analog.RIGHT_TRIGGER);
        left_bumper = get(Controls.LEFT_BUMPER);
        right_bumper = get(Controls.RIGHT_BUMPER);
        dpad_up = get(Controls.DPAD_UP);
        dpad_down = get(Controls.DPAD_DOWN);
        dpad_left = get(Controls.DPAD_LEFT);
        dpad_right = get(Controls.DPAD_RIGHT);
        left_stick_button = get(Controls.LEFT_STICK_BUTTON);
        right_stick_button = get(Controls.RIGHT_STICK_BUTTON);
        a = get(Controls.A);
        b = get(Controls.B);
        x = get(Controls.X);
        y = get(Controls.Y);
        start = get(Controls.START);
        back = get(Controls.BACK);

        updateButtonAliases();
    }

    @Override
    protected void updateButtonAliases() {
        super.updateButtonAliases();
        // Assign public field aliases
        lsx = left_stick_x;
        lsy = left_stick_y;
        rsx = right_stick_x;
        rsy = right_stick_y;
        lt = left_trigger;
        rt = right_trigger;
        lb = left_bumper;
        rb = right_bumper;
        du = dpad_up;
        dd = dpad_down;
        dl = dpad_left;
        dr = dpad_right;
        lsb = left_stick_button;
        rsb = right_stick_button;
    }

    /**
     * Customise how a button is read.
     *
     * @param button    The button to customise
     * @param predicate The custom function to use based on the button's value
     * @return this
     */
    public Controller set(Controls button, @Nullable Predicate<Boolean> predicate) {
        if (predicate == null) {
            buttons.remove(button);
            return this;
        }
        if (button != Controls.NONE)
            buttons.put(button, predicate);
        return this;
    }

    /**
     * Customise how an axis is read.
     *
     * @param axis     The axis to customise
     * @param function The custom function to use based on the axis's value
     * @return this
     */
    public Controller set(Controls.Analog axis, @Nullable Function<Float, Float> function) {
        if (function == null) {
            axes.remove(axis);
            return this;
        }
        axes.put(axis, function);
        return this;
    }

    /**
     * Customise how a group of buttons is read.
     *
     * @param group     The group of buttons to customise
     * @param predicate The custom function to use based on the group's value
     * @return this
     */
    public Controller set(Controls.ButtonGroup group, @Nullable Predicate<Boolean> predicate) {
        for (Controls button : getButtons(group)) {
            set(button, predicate);
        }
        return this;
    }

    /**
     * Customise how a group of axes is read.
     *
     * @param group    The group of axes to customise
     * @param function The custom function to use based on the group's value
     * @return this
     */
    public Controller set(Controls.AnalogGroup group, @Nullable Function<Float, Float> function) {
        for (Controls.Analog axis : getAxes(group)) {
            set(axis, function);
        }
        return this;
    }

    /**
     * Get the value of a button.
     *
     * @param button The button to get the value of
     * @return The value of the button
     */
    public boolean get(Controls button) {
        boolean isPressed = Controls.isSelected(sdk, button);
        Predicate<Boolean> predicate = buttons.get(button);
        return predicate == null ? isPressed : predicate.test(isPressed);
    }

    /**
     * Get the value of an axis.
     *
     * @param axis The axis to get the value of
     * @return The value of the axis
     */
    public float get(Controls.Analog axis) {
        float value = Controls.Analog.get(sdk, axis);
        Function<Float, Float> function = axes.get(axis);
        return function == null ? value : function.apply(value);
    }

    /**
     * Check if a button is currently pressed on a gamepad, with debounce to ignore a press that was already detected
     * upon the first call of this function and button.
     *
     * @param button The button to check
     * @return True if the button is pressed and not debounced
     */
    public boolean getDebounced(Controls button) {
        boolean buttonPressed = get(button);
        // Default value will be true as it won't be in the map, to avoid debouncing a value that was never pressed
        boolean isPressed = Boolean.TRUE.equals(debounces.getOrDefault(button, true));
        if (buttonPressed && !isPressed) {
            debounces.put(button, true);
            return true;
        } else if (!buttonPressed) {
            debounces.put(button, false);
        }
        return false;
    }

}
