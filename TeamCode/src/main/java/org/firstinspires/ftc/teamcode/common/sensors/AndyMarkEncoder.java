package org.firstinspires.ftc.teamcode.common.sensors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Uses a regular AndyMark/REV motor encoder for position input
 */
public class AndyMarkEncoder implements OdometryEncoder
{
    private DcMotor motor;
    private int zeroOff;
    
    private int prevPos;
    private long prevSample;
    
    private boolean direction;
    
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
        if (elapsed > 5000000) // = 5 ms
        {
            int pos = motor.getCurrentPosition() - zeroOff;
            prevPos = pos;
            prevSample = System.nanoTime();
            return pos * (direction ? -1 : 1);
        }
        else
        {
            return prevPos * (direction ? -1 : 1);
        }
    }
    
    public void setDirection(DcMotorSimple.Direction direction)
    {
        this.direction = direction == DcMotorSimple.Direction.REVERSE;
    }
    
    public DcMotorSimple.Direction getDirection()
    {
        return direction ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }
    
    @Override
    public boolean error()
    {
        return false;
    }
}
