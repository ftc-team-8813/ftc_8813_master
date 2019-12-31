package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.DataStorage;

public class FoundationHook
{
    private Servo hookL, hookR;
    private double hookL_up, hookL_down;
    private double hookR_up, hookR_down;
    private boolean down = false;

    public FoundationHook(Servo hookL, Servo hookR, DataStorage positions)
    {
        this.hookL = hookL;
        this.hookR = hookR;
        
        //#position 'hook l' up
        //#position 'hook l' down
        //#position 'hook r' up
        //#position 'hook r' down
        hookL_up = positions.getDouble("hook l.up", 0);
        hookL_down = positions.getDouble("hook l.down", 0);
        hookR_up = positions.getDouble("hook r.up", 0);
        hookR_down = positions.getDouble("hook r.down", 0);
    }

    public void moveHookDown()
    {
        hookL.setPosition(hookL_down);
        hookR.setPosition(hookR_down);
        down = true;
    }

    public void moveHookUp()
    {
        hookL.setPosition(hookL_up);
        hookR.setPosition(hookR_up);
        down = false;
    }
    
    public boolean hookDown()
    {
        return down;
    }
    
    public void toggle()
    {
        if (down) moveHookUp();
        else moveHookDown();
    }
}
