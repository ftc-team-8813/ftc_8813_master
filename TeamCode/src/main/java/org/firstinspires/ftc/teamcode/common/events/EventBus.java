package org.firstinspires.ftc.teamcode.common.events;

import org.firstinspires.ftc.teamcode.common.events.handler.EventHandler;
import org.firstinspires.ftc.teamcode.common.events.listener.EventListener;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class EventBus
{
    private List<EventHandler> eventHandlers;
    private List<EventListener> subscribers;
    
    public EventBus()
    {
        eventHandlers = new ArrayList<>();
        subscribers = new ArrayList<>();
    }
    
    public void addEventHandler(EventHandler handler)
    {
        eventHandlers.add(handler);
    }
    
    public void removeEventHandler(EventHandler handler)
    {
        eventHandlers.remove(handler);
    }
    
    public void subscribe(EventListener subscriber)
    {
        subscribers.add(subscriber);
    }
    
    public void unsubscribe(EventListener subscriber)
    {
        subscribers.remove(subscriber);
    }
    
    public List<Event> processEvents()
    {
        List<Event> currentEvents = new ArrayList<>();
        for (EventHandler handler : eventHandlers)
        {
            currentEvents.addAll(handler.processEvents());
        }
        return currentEvents;
    }
    
    public void pushEvents(List<Event> events)
    {
        for (Event ev : events)
        {
            for (EventListener l : subscribers)
            {
                if (ev.getClass() == l.getEventType())
                {
                    l.processEvent(ev);
                }
            }
        }
    }
    
    public void loop()
    {
        pushEvents(processEvents());
    }
}
