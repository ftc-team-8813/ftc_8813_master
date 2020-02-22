package org.firstinspires.ftc.teamcode.common.events;

public class ButtonEvent extends GamepadEvent
{
    /**
     * Valid button ID for this event
     */
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
    
    /**
     * Button ID
     */
    public final int button;
    /**
     * Button state (true = pressed)
     */
    public final boolean state;
    
    public final String mapping;
    
    public ButtonEvent(int gamepad, int button, boolean state, String mapping)
    {
        super("ButtonEvent", gamepad);
        this.button = button;
        this.state = state;
        this.mapping = mapping;
    }
}
