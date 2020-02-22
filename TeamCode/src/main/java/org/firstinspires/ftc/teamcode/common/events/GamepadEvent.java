package org.firstinspires.ftc.teamcode.common.events;

public abstract class GamepadEvent extends Event
{
    public final int gamepad;
    
    public GamepadEvent(String name, int gamepad)
    {
        super(name);
        this.gamepad = gamepad;
    }
}
