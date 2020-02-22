package org.firstinspires.ftc.teamcode.common.events.handler;

import android.util.SparseArray;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.events.AxisEvent;
import org.firstinspires.ftc.teamcode.common.events.ButtonEvent;
import org.firstinspires.ftc.teamcode.common.events.GamepadEvent;
import org.firstinspires.ftc.teamcode.common.util.Logger;

import java.util.ArrayList;
import java.util.List;

public class GamepadEventHandler implements EventHandler<GamepadEvent>
{
    private Gamepad gamepad1, gamepad2;
    private GamepadMapping mapping;
    private double[] prev_gp1;
    private double[] prev_gp2;
    
    
    public GamepadEventHandler(Gamepad gamepad1, Gamepad gamepad2, GamepadMapping mapping)
    {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.mapping = mapping;
        prev_gp1 = new double[19];
        prev_gp2 = new double[19];
    }
    
    private double[] parseGamepad(Gamepad g)
    {
        double[] padinfo = new double[19];
        // Buttons--we have infinite memory
        if (g.a)                  padinfo[0] = 1;
        if (g.b)                  padinfo[1] = 1;
        if (g.x)                  padinfo[2] = 1;
        if (g.y)                  padinfo[3] = 1;
        if (g.guide)              padinfo[4] = 1;
        if (g.start)              padinfo[5] = 1;
        if (g.back)               padinfo[6] = 1;
        if (g.left_bumper)        padinfo[7] = 1;
        if (g.right_bumper)       padinfo[8] = 1;
        if (g.left_stick_button)  padinfo[9] = 1;
        if (g.right_stick_button) padinfo[10] = 1;
        // Joysticks
        padinfo[11] = g.left_stick_x;
        padinfo[12] = g.left_stick_y;
        padinfo[13] = g.right_stick_x;
        padinfo[14] = g.right_stick_y;
        padinfo[15] = g.left_trigger;
        padinfo[16] = g.right_trigger;
        // D-Pad
        if (g.dpad_left)       padinfo[17] = -1;
        else if (g.dpad_right) padinfo[17] = 1;
        if (g.dpad_up)         padinfo[18] = 1;
        else if (g.dpad_down)  padinfo[18] = -1;
        
        return padinfo;
    }
    
    @Override
    public List<GamepadEvent> processEvents()
    {
        List<GamepadEvent> currentEvents = new ArrayList<>();
        
        double[] gp1 = parseGamepad(gamepad1);
        double[] gp2 = parseGamepad(gamepad2);
        for (int i = 0; i < 11; i++)
        {
            if (prev_gp1[i] != gp1[i])
            {
                prev_gp1[i] = gp1[i];
                String map = mapping.getButtonAction(mapping.getButtonId(1, i));
                boolean state = (gp1[i] == 1);
                currentEvents.add(new ButtonEvent(1, i, state, map));
            }
            
            if (prev_gp2[i] != gp2[i])
            {
                prev_gp2[i] = gp2[i];
                String map = mapping.getButtonAction(mapping.getButtonId(2, i));
                boolean state = (gp2[i] == 1);
                currentEvents.add(new ButtonEvent(2, i, state, map));
            }
        }
        
        for (int i = 11; i < 19; i++)
        {
            int ax = i - 11;
            if (prev_gp1[i] != gp1[i])
            {
                prev_gp1[i] = gp1[i];
                String map = mapping.getAxisAction(mapping.getAxisId(1, ax));
                currentEvents.add(new AxisEvent(1, ax, gp1[i], map));
            }
    
            if (prev_gp2[i] != gp2[i])
            {
                prev_gp2[i] = gp2[i];
                String map = mapping.getAxisAction(mapping.getAxisId(2, ax));
                currentEvents.add(new AxisEvent(2, ax, gp2[i], map));
            }
        }
        
        return currentEvents;
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
