package org.firstinspires.ftc.teamcode.common.events.listener;

import org.firstinspires.ftc.teamcode.common.events.ButtonEvent;

public interface ButtonEventListener extends EventListener<ButtonEvent>
{
    void onPress(ButtonEvent ev);
    void onRelease(ButtonEvent ev);
    
    @Override
    default void processEvent(ButtonEvent ev)
    {
        if (ev.state) onPress(ev);
        else onRelease(ev);
    }
}

