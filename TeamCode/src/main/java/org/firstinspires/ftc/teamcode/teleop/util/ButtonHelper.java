package org.firstinspires.ftc.teamcode.teleop.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;

/**
 * ButtonHelper - Utility to detect 'pressing' events (change of state) for user-interface
 * applications
 */

public class ButtonHelper {
    private Gamepad gamepad;
    /*
    Buttons:
    dpad_up dpad_down dpad_left dpad_right a b x y guide start back left_bumper right_bumper
    left_stick_button right_stick_button (15)
     */
    private boolean[] buttons = new boolean[15];
    private boolean[] held = new boolean[15];
    /**
     * A button for use in the pressed(), released(), and pressing() methods
     */
    public static final int dpad_up            =  0,
                            dpad_down          =  1,
                            dpad_left          =  2,
                            dpad_right         =  3,
                            a                  =  4,
                            b                  =  5,
                            x                  =  6,
                            y                  =  7,
                            guide              =  8,
                            start              =  9,
                            back               = 10,
                            left_bumper        = 11,
                            right_bumper       = 12,
                            left_stick_button  = 13,
                            right_stick_button = 14;

    public ButtonHelper(final Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    private void update() {
        boolean[] buttons2 = Arrays.copyOf(buttons, 15);
        buttons[dpad_up] = gamepad.dpad_up;
        buttons[dpad_down] = gamepad.dpad_down;
        buttons[dpad_left] = gamepad.dpad_left;
        buttons[dpad_right] = gamepad.dpad_right;
        buttons[a] = gamepad.a;
        buttons[b] = gamepad.b;
        buttons[x] = gamepad.x;
        buttons[y] = gamepad.y;
        buttons[guide] = gamepad.guide;
        buttons[start] = gamepad.start;
        buttons[back] = gamepad.back;
        buttons[left_bumper] = gamepad.left_bumper;
        buttons[right_bumper] = gamepad.right_bumper;
        buttons[left_stick_button] = gamepad.left_stick_button;
        buttons[right_stick_button] = gamepad.right_stick_button;
        for (int i = 0; i < 15; i++) {
            held[i] = buttons[i] && buttons2[i];
        }
    }

    public boolean pressed(int idx) {
        update();
        return buttons[idx];
    }

    public boolean released(int idx) {
        return !pressed(idx);
    }

    public boolean pressing(int idx) {
        boolean value = buttons[idx] && !held[idx];
        return value;
    }
}
