package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.Gamepad;

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
    private final Gamepad sdkGamepad;
    private final HashMap<Controls, Predicate<Boolean>> buttons = new HashMap<>();
    private final HashMap<Controls.Analog, Function<Float, Float>> axes = new HashMap<>();
    /**
     * Shorthand for left_stick_x
     */
    public float lsx;
    /**
     * Shorthand for left_stick_y
     */
    public float lsy;
    /**
     * Shorthand for right_stick_x
     */
    public float rsx;
    /**
     * Shorthand for right_stick_y
     */
    public float rsy;
    /**
     * Shorthand for left_trigger
     */
    public float lt;
    /**
     * Shorthand for right_trigger
     */
    public float rt;
    /**
     * Shorthand for left_bumper
     */
    public boolean lb;
    /**
     * Shorthand for right_bumper
     */
    public boolean rb;
    /**
     * Shorthand for dpad_up
     */
    public boolean du;
    /**
     * Shorthand for dpad_down
     */
    public boolean dd;
    /**
     * Shorthand for dpad_left
     */
    public boolean dl;
    /**
     * Shorthand for dpad_right
     */
    public boolean dr;
    /**
     * Shorthand for left_stick_button
     */
    public boolean lsb;
    /**
     * Shorthand for right_stick_button
     */
    public boolean rsb;

    /**
     * Create a new Controller to manage.
     *
     * @param gamepad The Gamepad to wrap (gamepad1, gamepad2)
     */
    public Controller(Gamepad gamepad) {
        sdkGamepad = gamepad;
    }

    /**
     * Update the public fields of this Controller with the values from the wrapped Gamepad, performing calculations
     * on the inputs as specified by the user.
     * This method must be called in a loop of the OpMode to update the Controller's values. This is automatically
     * called in BunyipsOpMode on another thread.
     */
    public void update() {
        // Copy over all state changes from the SDK gamepad
        copy(sdkGamepad);

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

    private Controls[] getButtons(ButtonGroup group) {
        switch (group) {
            case BUMPERS:
                return new Controls[]{Controls.LEFT_BUMPER, Controls.RIGHT_BUMPER};
            case DPAD:
                return new Controls[]{Controls.DPAD_UP, Controls.DPAD_DOWN, Controls.DPAD_LEFT, Controls.DPAD_RIGHT};
            case BUTTONS:
                return new Controls[]{Controls.A, Controls.B, Controls.X, Controls.Y};
            case SPECIAL:
                return new Controls[]{Controls.START, Controls.BACK, Controls.LEFT_STICK_BUTTON, Controls.RIGHT_STICK_BUTTON};
            case ALL:
                return Controls.values();
            default:
                return new Controls[0];
        }
    }

    private Controls.Analog[] getAxes(AnalogGroup group) {
        switch (group) {
            case ANALOG_STICKS:
                return new Controls.Analog[]{Controls.Analog.LEFT_STICK_X, Controls.Analog.LEFT_STICK_Y, Controls.Analog.RIGHT_STICK_X, Controls.Analog.RIGHT_STICK_Y};
            case TRIGGERS:
                return new Controls.Analog[]{Controls.Analog.LEFT_TRIGGER, Controls.Analog.RIGHT_TRIGGER};
            case ALL:
                // TODO: enums are borked
                return Controls.Analog.values();
            default:
                return new Controls.Analog[0];
        }
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
    public Controller set(ButtonGroup group, @Nullable Predicate<Boolean> predicate) {
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
    public Controller set(AnalogGroup group, @Nullable Function<Float, Float> function) {
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
        boolean isPressed = Controls.isSelected(this, button);
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
        float value = Controls.Analog.get(this, axis);
        Function<Float, Float> function = axes.get(axis);
        return function == null ? value : function.apply(value);
    }

    /**
     * Groups of buttons that can be customised.
     */
    public enum ButtonGroup {
        /**
         * All bumpers
         */
        BUMPERS,
        /**
         * All face buttons
         */
        DPAD,
        /**
         * ABXY buttons
         */
        BUTTONS,
        /**
         * Start, back, and stick buttons
         */
        SPECIAL,
        /**
         * All buttons
         */
        ALL
    }

    /**
     * Groups of axes that can be customised.
     */
    public enum AnalogGroup {
        /**
         * Both analog sticks
         */
        ANALOG_STICKS,
        /**
         * Both triggers
         */
        TRIGGERS,
        /**
         * All analog inputs
         */
        ALL
    }
}
