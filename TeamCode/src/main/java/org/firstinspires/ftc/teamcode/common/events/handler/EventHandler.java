package org.firstinspires.ftc.teamcode.common.events.handler;

import org.firstinspires.ftc.teamcode.common.events.Event;

import java.util.List;

public interface EventHandler<T extends Event>
{
    public List<T> processEvents();
}
