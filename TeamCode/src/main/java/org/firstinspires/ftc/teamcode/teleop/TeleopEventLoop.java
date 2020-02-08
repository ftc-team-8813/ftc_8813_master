package org.firstinspires.ftc.teamcode.teleop;

import android.util.SparseArray;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.teleop.event.ButtonEvent;
import org.firstinspires.ftc.teamcode.teleop.event.Event;
import org.firstinspires.ftc.teamcode.teleop.event.GamepadEvent;
import org.firstinspires.ftc.teamcode.teleop.event.listener.AxisEventListener;
import org.firstinspires.ftc.teamcode.teleop.event.listener.ButtonEventListener;
import org.firstinspires.ftc.teamcode.teleop.event.listener.EventListener;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class TeleopEventLoop
{
    private Gamepad gamepad1, gamepad2;
    private GamepadMapping mapping;
    
    private GamepadEventHandler gamepadHandler;
    
    public TeleopEventLoop(Gamepad gamepad1, Gamepad gamepad2, GamepadMapping mapping)
    {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
    
    public void subscribe(ButtonEventListener listener, String... buttons)
    {
        EventListenerHandler<ButtonEventListener> handler = new EventListenerHandler<>(listener, buttons);
        gamepadHandler.buttonListeners.add(handler);
    }
    
    public void subscribe(AxisEventListener listener, String... axes)
    {
        EventListenerHandler<AxisEventListener> handler = new EventListenerHandler<>(listener, axes);
        gamepadHandler.axisListeners.add(handler);
    }
    
    private class GamepadEventHandler
    {
        private List<EventListenerHandler<ButtonEventListener>> buttonListeners;
        private List<EventListenerHandler<AxisEventListener>> axisListeners;
        
        GamepadEventHandler()
        {
            buttonListeners = new ArrayList<>();
            axisListeners = new ArrayList<>();
        }
        
        public void processEvent(GamepadEvent ev)
        {
            if (ev instanceof ButtonEvent)
            {
                ButtonEvent e = (ButtonEvent)ev;
                
                String action = mapping.getButtonAction(mapping.getButtonId(e.gamepad, e.button));
                for (EventListenerHandler<ButtonEventListener> l : buttonListeners)
                {
                    if (l.listening(action))
                    {
                        if (e.state)
                        {
                            l.listener.onPress(e);
                        }
                        else
                        {
                            l.listener.onRelease(e);
                        }
                    }
                }
            }
        }
    }
    
    private class EventListenerHandler<T extends EventListener>
    {
        final T listener;
        final String[] filter;
        
        EventListenerHandler(T listener, String[] filter)
        {
            this.listener = listener;
            this.filter = filter;
        }
        
        boolean listening(String action)
        {
            for (String s : filter)
            {
                if (action.equals(s)) return true;
            }
            return false;
        }
    }
    
    public static class GamepadMapping
    {
        private SparseArray<String> buttonMapping;
        private SparseArray<String> axisMapping;
        private Logger log = new Logger("GamepadMapping");
        
        public GamepadMapping()
        {
            buttonMapping = new SparseArray<>();
            axisMapping = new SparseArray<>();
        }
        
        public int getButtonId(int gamepad, int button)
        {
            return (gamepad - 1) * 11 + button;
        }
        
        public int getAxisId(int gamepad, int axis)
        {
            return (gamepad - 1) * 8 + axis;
        }
        
        public void mapButton(int gamepad, int button, String action)
        {
            mapButton(getButtonId(gamepad, button), action);
        }
        
        public void mapButton(int button, String action)
        {
            if (buttonMapping.get(button) != null)
            {
                log.d("Remapping button %d to '%s' (was '%s')", button, action, buttonMapping.get(button));
            }
            buttonMapping.put(button, action);
        }
        
        public void mapAxis(int gamepad, int axis, String action)
        {
            mapAxis(getAxisId(gamepad, axis), action);
        }
        
        public void mapAxis(int axis, String action)
        {
            if (axisMapping.get(axis) != null)
            {
                log.d("Remapping axis %d to '%s' (was '%s')", axis, action, axisMapping.get(axis));
            }
            axisMapping.put(axis, action);
        }
        
        public String getButtonAction(int button)
        {
            return buttonMapping.get(button);
        }
        
        public String getAxisAction(int axis)
        {
            return axisMapping.get(axis);
        }
    }
}
