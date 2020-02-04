package org.firstinspires.ftc.teamcode.teleop.event.listener;

import org.firstinspires.ftc.teamcode.teleop.event.ButtonEvent;

public interface ButtonEventListener extends EventListener
{
    void onPress(ButtonEvent ev);
    void onRelease(ButtonEvent ev);
}
