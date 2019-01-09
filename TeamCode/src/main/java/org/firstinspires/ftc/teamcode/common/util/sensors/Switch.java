package org.firstinspires.ftc.teamcode.common.util.sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * A wrapper around DigitalChannel to implement a physical button switch.
 */
public class Switch
{
    private final DigitalChannel channel;
    private boolean inverted;
    private boolean lastState;

    public Switch(DigitalChannel channel)
    {
        this(channel, true);
    }

    public Switch(DigitalChannel channel, boolean inverted)
    {
        this.channel = channel;
        this.inverted = inverted;
    }


    public boolean pressed()
    {
        if (inverted) return !channel.getState();
        else return channel.getState();
    }

    public boolean released()
    {
        return !pressed();
    }

    public boolean pressing()
    {
        if (pressed())
        {
            if (!lastState)
            {
                lastState = true;
                return true;
            }
        }
        else
        {
            lastState = false;
        }
        return false;
    }
}
