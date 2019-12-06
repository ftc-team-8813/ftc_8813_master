package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.DataStorage;

public class Arm
{
    private Servo extension, claw;
    private double extension_min, extension_max;
    private boolean ext_reverse = false;
    private double claw_open, claw_closed;
    private boolean claw_state;
    public Arm(Servo extension, Servo claw, DataStorage positions)
    {
        this.extension = extension;
        this.claw = claw;
    
        this.extension_min = positions.getDouble("extension.min", 0);
        this.extension_max = positions.getDouble("extension.max", 1);
        this.claw_open = positions.getDouble("claw.open", 1);
        this.claw_closed = positions.getDouble("claw.closed", 0);
    
        if (extension_min > extension_max)
        {
            // Swap them
            double temp = extension_max;
            extension_max = extension_min;
            extension_min = temp;
            ext_reverse = true;
        }
        
        this.extension.scaleRange(extension_min, extension_max);
        if (ext_reverse)
        {
            this.extension.setPosition(1);
        }
        else
        {
            this.extension.setPosition(0);
        }
    }
    
    public void extend(double delta)
    {
        if (ext_reverse) delta = -delta;
        extension.setPosition(extension.getPosition() + delta);
    }
    
    public void closeClaw()
    {
        claw.setPosition(claw_closed);
        claw_state = true;
    }
    
    public void openClaw()
    {
        claw.setPosition(claw_open);
        claw_state = false;
    }
    
    public boolean clawClosed()
    {
        return claw_state;
    }
    
    public void toggleClaw()
    {
        if (claw_state) openClaw();
        else closeClaw();
    }

    public Servo getExtension(){
        return extension;
    }
}
