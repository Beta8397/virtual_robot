/*
 * Copyright (c) 2014, 2015 Qualcomm Technologies Inc
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Qualcomm Technologies Inc nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
Modified by Virtual_Robot contributors
 */

package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.util.Range;
import com.studiohartman.jamepad.ControllerState;

import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.ArrayBlockingQueue;

import virtual_robot.controller.VirtualGamePadController;

/**
 * Represents the GamePad.
 *
 * Note: the fields in the class are all public, but they should not be changed from the OpMode code.
 */
public class Gamepad {

    //for ps4
    public volatile boolean circle = false;
    public volatile boolean cross = false;
    public volatile boolean triangle = false;
    public volatile boolean square = false;
    public volatile boolean share = false;
    public volatile boolean options = false;
    public volatile boolean ps = false;

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

    public static int x_bit = 1;
    public static int y_bit = 1 << 1;
    public static int a_bit = 1 << 2;
    public static int b_bit = 1 << 3;
    public static int dpad_up_bit = 1 << 4;
    public static int dpad_down_bit = 1 << 5;
    public static int dpad_left_bit = 1 << 6;
    public static int dpad_right_bit = 1 << 7;
    public static int back_bit = 1 << 8;
    public static int guide_bit = 1 << 9;
    public static int start_bit = 1 << 10;
    public static int left_bumper_bit = 1 << 11;
    public static int right_bumper_bit = 1 << 12;
    public static int left_stick_button_bit = 1 << 13;
    public static int right_stick_button_bit = 1 << 14;
    public static int circle_bit = b_bit;
    public static int cross_bit = a_bit;
    public static int triangle_bit = y_bit;
    public static int square_bit = x_bit;
    public static int share_bit = back_bit;
    public static int options_bit = start_bit;
    public static int ps_bit = guide_bit;

    private int previous_bits = 0;

    /*
     * NOTE CHANGE IN DEADZONE TO MATCH NEW BEHAVIOR IN FTC SDK v. 7.0
     * There is no longer a setJoystickDeadzone method.
     */

    protected float deadzone = 0.2f;

    private float setWithDeadzone(float in) {
        float absIn = Math.abs(in);

        if (absIn > deadzone) {
//            return in;
            float absResult = (absIn - deadzone) / (1.0f - deadzone);
            return in > 0? absResult : -absResult;
        }

        return (float) 0.0;
    }

    public void resetValues(){
        x = false;
        y = false;
        a = false;
        b = false;
        left_stick_x = 0;
        left_stick_y = 0;
        right_stick_x = 0;
        right_stick_y = 0;
        dpad_up = false;
        dpad_down = false;
        dpad_left = false;
        dpad_right = false;
        back = false;
        guide = false;
        start = false;
        left_bumper = false;
        right_bumper = false;
        left_stick_button = false;
        right_stick_button = false;
        left_trigger = 0;
        right_trigger = 0;
    }

    public void update(ControllerState state){
        x = state.x;
        y = state.y;
        a = state.a;
        b = state.b;
        left_stick_x = setWithDeadzone(state.leftStickX);
        left_stick_y = setWithDeadzone(-state.leftStickY);
        right_stick_x = setWithDeadzone(state.rightStickX);
        right_stick_y = setWithDeadzone(-state.rightStickY);
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
        left_trigger = setWithDeadzone(state.leftTrigger);
        right_trigger = setWithDeadzone(state.rightTrigger);
        updateButtonAliases();
    }

    public void update(VirtualGamePadController.ControllerState state){
        x = state.x;
        y = state.y;
        a = state.a;
        b = state.b;
        left_stick_x = setWithDeadzone(state.leftStickX);
        left_stick_y = setWithDeadzone(state.leftStickY);
        right_stick_x = setWithDeadzone(state.rightStickX);
        right_stick_y = setWithDeadzone(state.rightStickY);
        dpad_up = state.dpad_up;
        dpad_down = state.dpad_down;
        dpad_left = state.dpad_left;
        dpad_right = state.dpad_right;
        back = false;
        guide = false;
        start = false;
        left_bumper = state.left_bumper;
        right_bumper = state.right_bumper;
        left_stick_button = false;
        right_stick_button = false;
        left_trigger = state.left_trigger;
        right_trigger = state.right_trigger;
        updateButtonAliases();
    }

    /**
     * Alias buttons so that XBOX & PS4 native button labels can be used in use code.
     * Should allow a team to program with whatever controllers they prefer, but
     * be able swap controllers easily without changing code.
     */
    protected void updateButtonAliases() {
        // There is no assignment for touchpad because there is no equivalent on XBOX controllers.
        circle = b;
        cross = a;
        triangle = y;
        square = x;
        share = back;
        options = start;
        ps = guide;
    }

    // To prevent blowing up the command queue if the user tries to send an LED command in a tight loop,
    // we have a 1-element evicting blocking queue for the outgoing LED effect and the event loop periodically
    // just grabs the effect out of it (if any)
    public EvictingBlockingQueue<LedEffect> ledQueue = new EvictingBlockingQueue<>(new ArrayBlockingQueue<LedEffect>(1));

    public static final int ID_UNASSOCIATED = -1;
    protected byte userForEffects = ID_UNASSOCIATED;

    public void setUserForEffects(byte userForEffects) {
        this.userForEffects = userForEffects;
    }

    public static final int LED_DURATION_CONTINUOUS = -1;

    public static class LedEffect {
        public static class Step {
            public int r;
            public int g;
            public int b;
            public int duration;
        }

        public final ArrayList<Step> steps;
        public final boolean repeating;
        public int user;

        private LedEffect(ArrayList<Step> steps, boolean repeating) {
            this.steps = steps;
            this.repeating = repeating;
        }

        public static class Builder {
            private ArrayList<Step> steps = new ArrayList<>();
            private boolean repeating;

            /**
             * Add a "step" to this LED effect. A step basically just means to set
             * the LED to a certain color (r,g,b) for a certain duration. By creating a chain of
             * steps, you can create unique effects.
             *
             * @param r the red LED intensity (0.0 - 1.0)
             * @param g the green LED intensity (0.0 - 1.0)
             * @param b the blue LED intensity (0.0 - 1.0)
             * @return the builder object, to follow the builder pattern
             */
            public Builder addStep(double r, double g, double b, int durationMs) {
                return addStepInternal(r, g, b, Math.max(durationMs, 0));
            }

            /**
             * Set whether this LED effect should loop after finishing,
             * unless the LED is otherwise commanded differently.
             *
             * @param repeating whether to loop
             * @return the builder object, to follow the builder pattern
             */
            public Builder setRepeating(boolean repeating) {
                this.repeating = repeating;
                return this;
            }

            private Builder addStepInternal(double r, double g, double b, int duration) {

                r = Range.clip(r, 0, 1);
                g = Range.clip(g, 0, 1);
                b = Range.clip(b, 0, 1);

                Step step = new Step();
                step.r = (int) Math.round(Range.scale(r, 0.0, 1.0, 0, 255));
                step.g = (int) Math.round(Range.scale(g, 0.0, 1.0, 0, 255));
                step.b = (int) Math.round(Range.scale(b, 0.0, 1.0, 0, 255));
                step.duration = duration;
                steps.add(step);
                return this;
            }

            /**
             * After you've added your steps, call this to get an LedEffect object
             * that you can then pass to {@link #runLedEffect(LedEffect)}
             *
             * @return an LedEffect object, built from previously added steps
             */
            public LedEffect build() {
                return new LedEffect(steps, repeating);
            }
        }
    }

    public void setLedColor(double r, double g, double b, int durationMs) {

        if (durationMs != LED_DURATION_CONTINUOUS) {
            durationMs = Math.max(0, durationMs);
        }

        LedEffect effect = new LedEffect.Builder().addStepInternal(r, g, b, durationMs).build();
        queueEffect(effect);
    }

    /**
     * Run an LED effect built using {@link LedEffect.Builder}
     * The LED effect will be run asynchronously; your OpMode will
     * not halt execution while the effect is running.
     * <p>
     * Calling this will displace any currently running LED effect
     */
    public void runLedEffect(LedEffect effect) {
        queueEffect(effect);
    }

    private void queueEffect(LedEffect effect) {
        // We need to make a new object here, because since the effect is queued and
        // not set immediately, if you called this function for two different gamepads
        // but passed in the same instance of an LED effect object, then it could happen
        // that both effect commands would be directed to the *same* gamepad. (Because of
        // the "user" field being modified before the queued command was serialized)
        LedEffect copy = new LedEffect(effect.steps, effect.repeating);
        copy.user = userForEffects;
        ledQueue.offer(copy);
    }

    // To prevent blowing up the command queue if the user tries to send a rumble command in a tight loop,
    // we have a 1-element evicting blocking queue for the outgoing rumble effect and the event loop periodically
    // just grabs the effect out of it (if any)
    public EvictingBlockingQueue<RumbleEffect> rumbleQueue = new EvictingBlockingQueue<>(new ArrayBlockingQueue<RumbleEffect>(1));
    public long nextRumbleApproxFinishTime = RUMBLE_FINISH_TIME_FLAG_NOT_RUMBLING;

    public static final int RUMBLE_DURATION_CONTINUOUS = -1;

    public static class RumbleEffect {
        public static class Step {
            public int large;
            public int small;
            public int duration;
        }

        public int user;
        public final ArrayList<Step> steps;

        private RumbleEffect(ArrayList<Step> steps) {
            this.steps = steps;
        }


        public static class Builder {
            private ArrayList<Step> steps = new ArrayList<>();

            /**
             * Add a "step" to this rumble effect. A step basically just means to rumble
             * at a certain power level for a certain duration. By creating a chain of
             * steps, you can create unique effects. See {@link #rumbleBlips(int)} for a
             * a simple example.
             *
             * @param rumble1    rumble power for rumble motor 1 (0.0 - 1.0)
             * @param rumble2    rumble power for rumble motor 2 (0.0 - 1.0)
             * @param durationMs milliseconds this step lasts
             * @return the builder object, to follow the builder pattern
             */
            public Builder addStep(double rumble1, double rumble2, int durationMs) {
                return addStepInternal(rumble1, rumble2, Math.max(durationMs, 0));
            }

            private Builder addStepInternal(double rumble1, double rumble2, int durationMs) {

                rumble1 = Range.clip(rumble1, 0, 1);
                rumble2 = Range.clip(rumble2, 0, 1);

                Step step = new Step();
                step.large = (int) Math.round(Range.scale(rumble1, 0.0, 1.0, 0, 255));
                step.small = (int) Math.round(Range.scale(rumble2, 0.0, 1.0, 0, 255));
                step.duration = durationMs;
                steps.add(step);

                return this;
            }

            /**
             * After you've added your steps, call this to get a RumbleEffect object
             * that you can then pass to {@link #runRumbleEffect(RumbleEffect)}
             *
             * @return a RumbleEffect object, built from previously added steps
             */
            public RumbleEffect build() {
                return new RumbleEffect(steps);
            }
        }
    }

    /**
     * Run a rumble effect built using {@link RumbleEffect.Builder}
     * The rumble effect will be run asynchronously; your OpMode will
     * not halt execution while the effect is running.
     * <p>
     * Calling this will displace any currently running rumble effect
     */
    public void runRumbleEffect(RumbleEffect effect) {
        queueEffect(effect);
    }

    /**
     * Rumble the gamepad's first rumble motor at maximum power for a certain duration.
     * Calling this will displace any currently running rumble effect.
     *
     * @param durationMs milliseconds to rumble for, or {@link #RUMBLE_DURATION_CONTINUOUS}
     */
    public void rumble(int durationMs) {

        if (durationMs != RUMBLE_DURATION_CONTINUOUS) {
            durationMs = Math.max(0, durationMs);
        }

        RumbleEffect effect = new RumbleEffect.Builder().addStepInternal(1.0, 0, durationMs).build();
        queueEffect(effect);
    }

    /**
     * Rumble the gamepad at a fixed rumble power for a certain duration
     * Calling this will displace any currently running rumble effect
     *
     * @param rumble1    rumble power for rumble motor 1 (0.0 - 1.0)
     * @param rumble2    rumble power for rumble motor 2 (0.0 - 1.0)
     * @param durationMs milliseconds to rumble for, or {@link #RUMBLE_DURATION_CONTINUOUS}
     */
    public void rumble(double rumble1, double rumble2, int durationMs) {

        if (durationMs != RUMBLE_DURATION_CONTINUOUS) {
            durationMs = Math.max(0, durationMs);
        }

        RumbleEffect effect = new RumbleEffect.Builder().addStepInternal(rumble1, rumble2, durationMs).build();
        queueEffect(effect);
    }

    /**
     * Cancel the currently running rumble effect, if any
     */
    public void stopRumble() {
        rumble(0, 0, RUMBLE_DURATION_CONTINUOUS);
    }

    /**
     * Rumble the gamepad for a certain number of "blips" using predetermined blip timing
     * This will displace any currently running rumble effect.
     *
     * @param count the number of rumble blips to perform
     */
    public void rumbleBlips(int count) {
        RumbleEffect.Builder builder = new RumbleEffect.Builder();

        for (int i = 0; i < count; i++) {
            builder.addStep(1.0, 0, 250).addStep(0, 0, 100);
        }

        queueEffect(builder.build());
    }

    private void queueEffect(RumbleEffect effect) {
        // We need to make a new object here, because since the effect is queued and
        // not set immediately, if you called this function for two different gamepads
        // but passed in the same instance of a rumble effect object, then it could happen
        // that both rumble commands would be directed to the *same* gamepad. (Because of
        // the "user" field being modified before the queued command was serialized)
        RumbleEffect copy = new RumbleEffect(effect.steps);
        copy.user = userForEffects;
        rumbleQueue.offer(copy);
        nextRumbleApproxFinishTime = calcApproxRumbleFinishTime(copy);
    }

    private static final long RUMBLE_FINISH_TIME_FLAG_NOT_RUMBLING = -1;
    private static final long RUMBLE_FINISH_TIME_FLAG_INFINITE = Long.MAX_VALUE;

    /**
     * Returns an educated guess about whether there is a rumble action ongoing on this gamepad
     *
     * @return an educated guess about whether there is a rumble action ongoing on this gamepad
     */
    public boolean isRumbling() {
        if (nextRumbleApproxFinishTime == RUMBLE_FINISH_TIME_FLAG_NOT_RUMBLING) {
            return false;
        } else if (nextRumbleApproxFinishTime == RUMBLE_FINISH_TIME_FLAG_INFINITE) {
            return true;
        } else {
            return System.currentTimeMillis() < nextRumbleApproxFinishTime;
        }
    }

    private long calcApproxRumbleFinishTime(RumbleEffect effect) {
        // If the effect is only 1 step long and has an infinite duration...
        if (effect.steps.size() == 1 && effect.steps.get(0).duration == RUMBLE_DURATION_CONTINUOUS) {
            // If the power is zero, then that means the gamepad is being commanded to cease rumble
            if (effect.steps.get(0).large == 0 && effect.steps.get(0).small == 0) {
                return RUMBLE_FINISH_TIME_FLAG_NOT_RUMBLING;
            } else { // But if not, that means it's an infinite (continuous) rumble command
                return RUMBLE_FINISH_TIME_FLAG_INFINITE;
            }
        } else { // If the effect has more than one step (or one step with non-infinite duration) we need to sum the step times
            long time = System.currentTimeMillis();
            long overhead = 50 /* rumbleGamepadsInterval in FtcEventLoopHandler */ +
                    //AGS: not on simulator - SendOnceRunnable.MS_BATCH_TRANSMISSION_INTERVAL +
                    5  /* Slop */;
            for (RumbleEffect.Step step : effect.steps) {
                time += step.duration;
            }
            time += overhead;
            return time;
        }
    }

    public int buttonToBit(boolean val, int bit){
        if (val) return bit;
        else return 0;
    }

    public int buttonsToInt(){
        int result = 0;
        boolean[] buttons = new boolean[]{x, y, a, b, dpad_up, dpad_down, dpad_left, dpad_right,
                back, guide, start, left_bumper, right_bumper, left_stick_button, right_stick_button};
        for (int i=0; i<buttons.length; i++){
            result |= buttonToBit(buttons[i], 1<<i);
        }
        return result;
    }

    public synchronized void resetEdgeDetection() {
        previous_bits = buttonsToInt();
    }

    public synchronized boolean wasButtonPressed(boolean val, int bit) {
        boolean result = val &&  (previous_bits & bit) == 0;
        if (result) {
            previous_bits |= bit;
        }
        return result;
    }

    public synchronized boolean wasButtonReleased(boolean val, int bit) {
        boolean result = !val && (previous_bits & bit) != 0;
        if (result) {
            previous_bits &= ~bit;
        }
        return result;
    }


    /**
     * Checks if dpad_up was pressed since the last call of this method
     * @return true if dpad_up was pressed since the last call of this method; otherwise false
     */
    public boolean dpadUpWasPressed() {
        return wasButtonPressed(dpad_up, dpad_up_bit);
    }

    /**
     * Checks if dpad_up was released since the last call of this method
     * @return true if dpad_up was released since the last call of this method; otherwise false
     */
    public boolean dpadUpWasReleased() {
        return wasButtonReleased(dpad_up, dpad_up_bit);
    }

    /**
     * Checks if dpad_down was pressed since the last call of this method
     * @return true if dpad_down was pressed since the last call of this method; otherwise false
     */
    public boolean dpadDownWasPressed() {
        return wasButtonPressed(dpad_down, dpad_down_bit);
    }

    /**
     * Checks if dpad_down was released since the last call of this method
     * @return true if dpad_down was released since the last call of this method; otherwise false
     */
    public boolean dpadDownWasReleased() {
        return wasButtonReleased(dpad_down, dpad_down_bit);
    }

    /**
     * Checks if dpad_left was pressed since the last call of this method
     * @return true if dpad_left was pressed since the last call of this method; otherwise false
     */
    public boolean dpadLeftWasPressed() {
        return wasButtonPressed(dpad_left, dpad_left_bit);
    }

    /**
     * Checks if dpad_left was released since the last call of this method
     * @return true if dpad_left was released since the last call of this method; otherwise false
     */
    public boolean dpadLeftWasReleased() {
        return wasButtonReleased(dpad_left, dpad_left_bit);
    }

    /**
     * Checks if dpad_right was pressed since the last call of this method
     * @return true if dpad_right was pressed since the last call of this method; otherwise false
     */
    public boolean dpadRightWasPressed() {
        return wasButtonPressed(dpad_right, dpad_right_bit);
    }

    /**
     * Checks if dpad_right was released since the last call of this methmethodod
     * @return true if dpad_right was released since the last call of this ; otherwise false
     */
    public boolean dpadRightWasReleased() {
        return wasButtonReleased(dpad_right, dpad_right_bit);
    }

    /**
     * Checks if a was pressed since the last call of this method
     * @return true if a was pressed since the last call of this method; otherwise false
     */
    public boolean aWasPressed() {
        return wasButtonPressed(a, a_bit);
    }

    /**
     * Checks if a was released since the last call of this method
     * @return true if a was released since the last call of this method; otherwise false
     */
    public boolean aWasReleased() {
        return wasButtonReleased(a, a_bit);
    }

    /**
     * Checks if b was pressed since the last call of this method
     * @return true if b was pressed since the last call of this method; otherwise false
     */
    public boolean bWasPressed() {
        return wasButtonPressed(b, b_bit);
    }

    /**
     * Checks if b was released since the last call of this method
     * @return true if b was released since the last call of this method; otherwise false
     */
    public boolean bWasReleased() {
        return wasButtonReleased(b, b_bit);
    }

    /**
     * Checks if x was pressed since the last call of this method
     * @return true if x was pressed since the last call of this method; otherwise false
     */
    public boolean xWasPressed() {
        return wasButtonPressed(x, x_bit);
    }

    /**
     * Checks if x was released since the last call of this method
     * @return true if x was released since the last call of this method; otherwise false
     */
    public boolean xWasReleased() {
        return wasButtonReleased(x, x_bit);
    }

    /**
     * Checks if y was pressed since the last call of this method
     * @return true if y was pressed since the last call of this method; otherwise false
     */
    public boolean yWasPressed() {
        return wasButtonPressed(y, y_bit);
    }

    /**
     * Checks if y was released since the last call of this method
     * @return true if y was released since the last call of this method; otherwise false
     */
    public boolean yWasReleased() {
        return wasButtonReleased(y, y_bit);
    }

    /**
     * Checks if guide was pressed since the last call of this method
     * @return true if guide was pressed since the last call of this method; otherwise false
     */
    public boolean guideWasPressed() {
        return wasButtonPressed(guide, guide_bit);
    }

    /**
     * Checks if guide was released since the last call of this method
     * @return true if guide was released since the last call of this method; otherwise false
     */
    public boolean guideWasReleased() {
        return wasButtonReleased(guide, guide_bit);
    }

    /**
     * Checks if start was pressed since the last call of this method
     * @return true if start was pressed since the last call of this method; otherwise false
     */
    public boolean startWasPressed() {
        return wasButtonPressed(start, start_bit);
    }

    /**
     * Checks if start was released since the last call of this method
     * @return true if start was released since the last call of this method; otherwise false
     */
    public boolean startWasReleased() {
        return wasButtonReleased(start, start_bit);
    }

    /**
     * Checks if back was pressed since the last call of this method
     * @return true if back was pressed since the last call of this method; otherwise false
     */
    public boolean backWasPressed() {
        return wasButtonPressed(back, back_bit);
    }

    /**
     * Checks if back was released since the last call of this method
     * @return true if back was released since the last call of this method; otherwise false
     */
    public boolean backWasReleased() {
        return wasButtonReleased(back, back_bit);
    }

    /**
     * Checks if left_bumper was pressed since the last call of this method
     * @return true if left_bumper was pressed since the last call of this method; otherwise false
     */
    public boolean leftBumperWasPressed() {
        return wasButtonPressed(left_bumper, left_bumper_bit);
    }

    /**
     * Checks if left_bumper was released since the last call of this method
     * @return true if left_bumper was released since the last call of this method; otherwise false
     */
    public boolean leftBumperWasReleased() {
        return wasButtonReleased(left_bumper, left_bumper_bit);
    }

    /**
     * Checks if right_bumper was pressed since the last call of this method
     * @return true if right_bumper was pressed since the last call of this method; otherwise false
     */
    public boolean rightBumperWasPressed() {
        return wasButtonPressed(right_bumper, right_bumper_bit);
    }

    /**
     * Checks if right_bumper was released since the last call of this method
     * @return true if right_bumper was released since the last call of this method; otherwise false
     */
    public boolean rightBumperWasReleased() {
        return wasButtonReleased(right_bumper, right_bumper_bit);
    }

    /**
     * Checks if left_stick_button was pressed since the last call of this method
     * @return true if left_stick_button was pressed since the last call of this method; otherwise false
     */
    public boolean leftStickButtonWasPressed() {
        return wasButtonPressed(left_stick_button, left_stick_button_bit);
    }

    /**
     * Checks if left_stick_button was released since the last call of this method
     * @return true if left_stick_button was released since the last call of this method; otherwise false
     */
    public boolean leftStickButtonWasReleased() {
        return wasButtonReleased(left_stick_button, left_stick_button_bit);
    }

    /**
     * Checks if right_stick_button was pressed since the last call of this method
     * @return true if right_stick_button was pressed since the last call of this method; otherwise false
     */
    public boolean rightStickButtonWasPressed() {
        return wasButtonPressed(right_stick_button, right_stick_button_bit);
    }

    /**
     * Checks if right_stick_button was released since the last call of this method
     * @return true if right_stick_button was released since the last call of this method; otherwise false
     */
    public boolean rightStickButtonWasReleased() {
        return wasButtonReleased(right_stick_button, right_stick_button_bit);
    }

    /**
     * Checks if circle was pressed since the last call of this method
     * @return true if circle was pressed since the last call of this method; otherwise false
     */
    public boolean circleWasPressed() {
        return wasButtonPressed(circle, circle_bit);
    }

    /**
     * Checks if circle was released since the last call of this method
     * @return true if circle was released since the last call of this method; otherwise false
     */
    public boolean circleWasReleased() {
        return wasButtonReleased(circle, circle_bit);
    }

    /**
     * Checks if cross was pressed since the last call of this method
     * @return true if cross was pressed since the last call of this method; otherwise false
     */
    public boolean crossWasPressed() {
        return wasButtonPressed(cross, cross_bit);
    }

    /**
     * Checks if cross was released since the last call of this method
     * @return true if cross was released since the last call of this method; otherwise false
     */
    public boolean crossWasReleased() {
        return wasButtonReleased(cross, cross_bit);
    }

    /**
     * Checks if triangle was pressed since the last call of this method
     * @return true if triangle was pressed since the last call of this method; otherwise false
     */
    public boolean triangleWasPressed() {
        return wasButtonPressed(triangle, triangle_bit);
    }

    /**
     * Checks if triangle was released since the last call of this method
     * @return true if triangle was released since the last call of this method; otherwise false
     */
    public boolean triangleWasReleased() {
        return wasButtonReleased(triangle, triangle_bit);
    }

    /**
     * Checks if square was pressed since the last call of this method
     * @return true if square was pressed since the last call of this method; otherwise false
     */
    public boolean squareWasPressed() {
        return wasButtonPressed(square, square_bit);
    }

    /**
     * Checks if square was released since the last call of this method
     * @return true if square was released since the last call of this method; otherwise false
     */
    public boolean squareWasReleased() {
        return wasButtonReleased(square, square_bit);
    }

    /**
     * Checks if share was pressed since the last call of this method
     * @return true if share was pressed since the last call of this method; otherwise false
     */
    public boolean shareWasPressed() {
        return wasButtonPressed(share, share_bit);
    }

    /**
     * Checks if share was released since the last call of this method
     * @return true if share was released since the last call of this method; otherwise false
     */
    public boolean shareWasReleased() {
        return wasButtonReleased(share, share_bit);
    }

    /**
     * Checks if options was pressed since the last call of this method
     * @return true if options was pressed since the last call of this method; otherwise false
     */
    public boolean optionsWasPressed() {
        return wasButtonPressed(options, options_bit);
    }

    /**
     * Checks if options was released since the last call of this method
     * @return true if options was released since the last call of this method; otherwise false
     */
    public boolean optionsWasReleased() {
        return wasButtonReleased(options, options_bit);
    }


    /**
     * Checks if ps was pressed since the last call of this method
     * @return true if ps was pressed since the last call of this method; otherwise false
     */
    public boolean psWasPressed() {
        return wasButtonPressed(ps, ps_bit);
    }

    /**
     * Checks if ps was released since the last call of this method
     * @return true if ps was released since the last call of this method; otherwise false
     */
    public boolean psWasReleased() {
        return wasButtonReleased(ps, ps_bit);
    }
}
