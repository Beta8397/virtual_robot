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

import com.studiohartman.jamepad.ControllerState;
import virtual_robot.controller.VirtualGamePadController;

/**
 * Represents the GamePad.
 *
 * Note: the fields in the class are all public, but they should not be changed from the OpMode code.
 */
public class Gamepad {

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
    private volatile float deadzone = (float) 0.0;

    public void setJoystickDeadzone(float deadzone) {
        this.deadzone = deadzone;
    }

    private float setWithDeadzone(float in) {
        if (Math.abs(in) > deadzone) {
            return in;
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
    }

}
