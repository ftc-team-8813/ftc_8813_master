package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDColorCommand;

import org.firstinspires.ftc.teamcode.common.util.Logger;

public class RevHubLED
{
    private LynxModule hub;
    private Logger log;
    
    public RevHubLED(LynxModule module)
    {
        this.hub = module;
        log = new Logger("RevHub LED");
    }
    
    public void setColor(int red, int green, int blue)
    {
        LynxSetModuleLEDColorCommand cmd = new LynxSetModuleLEDColorCommand(hub, (byte)red, (byte)green, (byte)blue);
        try
        {
            cmd.send();
        } catch (InterruptedException | LynxNackException e)
        {
            log.w(e);
        }
    }
}
