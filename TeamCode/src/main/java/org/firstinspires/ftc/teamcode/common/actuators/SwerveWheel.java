package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;

public class SwerveWheel
{
    private PIDMotor upper, lower;
    
    public SwerveWheel(PIDMotor upper, PIDMotor lower)
    {
        this.upper = upper;
        this.lower = lower;
        
        this.upper.setDirection(DcMotorSimple.Direction.FORWARD);
        this.lower.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    
    public void setDirection(DcMotorSimple.Direction d)
    {
        this.upper.setDirection(d);
        this.lower.setDirection(d);
    }
    
    public void move(int encUpper, int encLower, double power) throws InterruptedException
    {
        upper.setPower(power);
        lower.setPower(power);
        
        upper.startRunToPosition(encUpper);
        lower.startRunToPosition(encLower);
        while (upper.isHolding() || lower.isHolding())
        {
            Thread.sleep(10);
        }
    }
    
    public void moveAsync(int encUpper, int encLower, double power)
    {
        upper.setPower(power);
        lower.setPower(power);
    
        upper.startRunToPosition(encUpper);
        lower.startRunToPosition(encLower);
    }
    
    public void drive(double pUpper, double pLower)
    {
        upper.getMotor().setPower(pUpper);
        lower.getMotor().setPower(pLower);
    }
    
    public void stop()
    {
        upper.stopHolding();
        lower.stopHolding();
    }
    
    public int getUpperPos()
    {
        return upper.getCurrentPosition();
    }
    
    public int getLowerPos()
    {
        return lower.getCurrentPosition();
    }
    
    public void resetEncoders()
    {
        upper.stopHolding();
        lower.stopHolding();
        
        upper.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lower.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
        upper.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lower.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    public void copy(final SwerveWheel other)
    {
        GlobalThreadPool.instance().start(() ->
        {
            upper.setPower(1);
            lower.setPower(1);
            while (true)
            {
                upper.hold(other.upper.getCurrentPosition());
                lower.hold(other.lower.getCurrentPosition());
                try
                {
                    Thread.sleep(20);
                }
                catch (InterruptedException e)
                {
                    break;
                }
            }
        });
    }
}
