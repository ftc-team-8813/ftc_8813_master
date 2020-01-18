package org.firstinspires.ftc.teamcode.common.sensors;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Uses a regular AndyMark/REV motor encoder for position input
 */
public class AndyMarkEncoder implements OdometryEncoder
{
    private DcMotor motor;
    private int zeroOff;
    
    private int prevPos;
    private long prevSample;
    
    public AndyMarkEncoder(DcMotor motor)
    {
        this.motor = motor;
        resetEncoder();
    }
    
    @Override
    public void resetEncoder()
    {
        zeroOff = motor.getCurrentPosition();
    }
    
    @Override
    public double getPosition()
    {
        long elapsed = System.nanoTime() - prevSample;
        if (elapsed > 5000000)
        {
            int pos = motor.getCurrentPosition() - zeroOff;
            prevPos = pos;
            prevSample = System.nanoTime();
            return pos;
        }
        else
        {
            return prevPos;
        }
    }
    
    @Override
    public boolean error()
    {
        return false;
    }
}
