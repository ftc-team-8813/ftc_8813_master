package org.firstinspires.ftc.teamcode.common.events.handler;

import org.firstinspires.ftc.teamcode.common.events.Event;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class TimeEventHandler implements EventHandler<TimeEventHandler.TimerEvent>
{
    private class Timer
    {
        long nanos;
        long end;
        boolean repeat;
        boolean dead;
        int id;
        
        Timer(long nanos, boolean repeat)
        {
            id = new Random().nextInt();
            dead = false;
            this.nanos = nanos;
            this.end = System.nanoTime() + nanos;
            this.repeat = repeat;
        }
        
        boolean check()
        {
            if (System.nanoTime() > end && !dead)
            {
                if (repeat)
                {
                    end = System.nanoTime() + nanos;
                }
                else
                {
                    dead = true;
                }
                return true;
            }
            return false;
        }
    }
    
    public class TimerEvent extends Event
    {
        public final int id;
        
        public TimerEvent(int id)
        {
            super("TimerEvent");
            this.id = id;
        }
    }
    
    private List<Timer> timers;
    
    public TimeEventHandler()
    {
        timers = new ArrayList<>();
    }
    
    public void addTimer(double seconds)
    {
        // TODO
    }
    
    @Override
    public List<TimerEvent> processEvents()
    {
        return null;
    }
}
