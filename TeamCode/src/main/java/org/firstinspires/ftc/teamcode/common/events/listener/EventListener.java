package org.firstinspires.ftc.teamcode.common.events.listener;


import org.firstinspires.ftc.teamcode.common.events.Event;

public interface EventListener<T extends Event>
{
    void processEvent(T event);
    
    Class<?> getEventType();
}
