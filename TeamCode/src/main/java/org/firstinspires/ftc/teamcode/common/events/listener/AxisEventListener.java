package org.firstinspires.ftc.teamcode.common.events.listener;

import org.firstinspires.ftc.teamcode.common.events.AxisEvent;

public interface AxisEventListener extends EventListener<AxisEvent>
{
    void onAxisChange(AxisEvent event);
    
    default void processEvent(AxisEvent ev)
    {
        onAxisChange(ev);
    }
}
