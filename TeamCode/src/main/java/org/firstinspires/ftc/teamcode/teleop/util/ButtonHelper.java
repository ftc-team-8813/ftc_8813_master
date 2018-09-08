package org.firstinspires.ftc.teamcode.teleop.util;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * ButtonHelper - Utility to detect 'pressing' events (change of state) for user-interface
 * applications
 */

public class ButtonHelper
{
    private Gamepad gamepad;
    /*
    Buttons:
    dpad_up dpad_down dpad_left dpad_right a b x y guide start back left_bumper right_bumper
    left_stick_button right_stick_button (15)
     */
    private boolean[] buttons = new boolean[15];
    /**
     * A button for use in the pressed(), released(), and pressing() methods
     */
    public static final int dpad_up = 0,
            dpad_down = 1,
            dpad_left = 2,
            dpad_right = 3,
            a = 4,
            b = 5,
            x = 6,
            y = 7,
            guide = 8,
            start = 9,
            back = 10,
            left_bumper = 11,
            right_bumper = 12,
            left_stick_button = 13,
            right_stick_button = 14;
    
    public ButtonHelper(final Gamepad gamepad)
    {
        this.gamepad = gamepad;
    }
    
    private boolean getButton(int button)
    {
        if (button == dpad_up) return gamepad.dpad_up;
        else if (button == dpad_down) return gamepad.dpad_down;
        else if (button == dpad_left) return gamepad.dpad_left;
        else if (button == dpad_right) return gamepad.dpad_right;
        else if (button == a) return gamepad.a;
        else if (button == b) return gamepad.b;
        else if (button == x) return gamepad.x;
        else if (button == y) return gamepad.y;
        else if (button == guide) return gamepad.guide;
        else if (button == start) return gamepad.start;
        else if (button == back) return gamepad.back;
        else if (button == left_bumper) return gamepad.left_bumper;
        else if (button == right_bumper) return gamepad.right_bumper;
        else if (button == left_stick_button) return gamepad.left_stick_button;
        else if (button == right_stick_button) return gamepad.right_stick_button;
        else return false;
    }
    
    public boolean pressed(int idx)
    {
        return getButton(idx);
    }
    
    public boolean released(int idx)
    {
        return !pressed(idx);
    }
    
    public boolean pressing(int idx)
    {
        boolean held = buttons[idx];
        buttons[idx] = pressed(idx);
        return !held && pressed(idx);
    }
}
