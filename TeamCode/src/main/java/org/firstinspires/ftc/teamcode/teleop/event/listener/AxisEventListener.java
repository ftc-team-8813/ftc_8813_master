package org.firstinspires.ftc.teamcode.teleop.event.listener;

import org.firstinspires.ftc.teamcode.teleop.event.AxisEvent;

public interface AxisEventListener extends EventListener
{
    void onAxisChange(AxisEvent event);
}
