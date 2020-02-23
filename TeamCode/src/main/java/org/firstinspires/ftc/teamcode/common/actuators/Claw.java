package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.teamcode.common.util.DataStorage;

public class Claw
{
    private Servo claw;
    private double claw_up, claw_open, claw_closed;
    private boolean claw_is_closed;
    private ServoController controller;
    private int port;
    
    public Claw(Servo claw, DataStorage positions)
    {
        this.claw = claw;
        this.port = claw.getPortNumber();
    
        //#position claw fullUp
        //#position claw open
        //#position claw closed
        this.claw_up = positions.getDouble("claw.fullUp", 1);
        this.claw_open = positions.getDouble("claw.open", 1);
        this.claw_closed = positions.getDouble("claw.closed", 0);
        
    }

    
    public void closeClaw()
    {
        claw.setPosition(claw_closed);
        claw_is_closed = true;
    }

    public void closeClawAsync(){
        controller.setServoPosition(port, claw_closed);
        claw_is_closed = true;
    }
    
    public void openClaw()
    {
        claw.setPosition(claw_open);
        claw_is_closed = false;
    }

    public void openClawAsync(){
        controller.setServoPosition(port, claw_open);
        claw_is_closed = false;
    }
    
    public void setClawUp()
    {
        claw.setPosition(claw_up);
        claw_is_closed = false; // Technically open
    }
    
    public boolean clawClosed()
    {
        return claw_is_closed;
    }
    
    public void toggleClaw()
    {
        if (claw_is_closed) openClaw();
        else closeClaw();
    }
}
