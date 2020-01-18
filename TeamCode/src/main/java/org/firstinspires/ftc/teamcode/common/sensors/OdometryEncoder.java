package org.firstinspires.ftc.teamcode.common.sensors;

public interface OdometryEncoder
{
    public void resetEncoder();
    
    public double getPosition();
    
    public boolean error();
}
