package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.Servo;

public class FoundationHook
{
    private Servo hook;
    private boolean down = false;

    public FoundationHook(Servo hook)
    {
        this.hook = hook;
    }

    public void moveHookDown()
    {
        hook.setPosition(0.8);
        down = true;
    }

    public void moveHookUp()
    {
        hook.setPosition(0);
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
