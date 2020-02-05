package org.firstinspires.ftc.teamcode.teleop.event;

public class ButtonEvent extends GamepadEvent
{
    public static final int
            a                  = 0,
            b                  = 1,
            x                  = 2,
            y                  = 3,
            guide              = 4,
            start              = 5,
            back               = 6,
            left_bumper        = 7,
            right_bumper       = 8,
            left_stick_button  = 9,
            right_stick_button = 10;
    
    public final int button;
    public final boolean state;
    
    public ButtonEvent(int gamepad, int button, boolean state)
    {
        super("ButtonEvent", gamepad);
        this.button = button;
        this.state = state;
    }
}
