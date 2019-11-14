package org.firstinspires.ftc.teamcode.common.motor_control;

import android.annotation.SuppressLint;
import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;

import java.util.concurrent.Future;

public class AccelMotor extends DcMotorImpl
{
    private double acceleration;
    private double maxSpeed;
    private Future<?> currentJob;
    private Logger log;
    private final double defaultAcceleration;
    
    public AccelMotor(DcMotor motor)
    {
        this(motor, motor.getMotorType().getAchieveableMaxTicksPerSecondRounded());
    }
    
    @SuppressLint("DefaultLocale")
    public AccelMotor(DcMotor motor, double acceleration)
    {
        super(motor.getController(), motor.getPortNumber(), motor.getDirection(), motor.getMotorType());
        maxSpeed = motor.getMotorType().getAchieveableMaxTicksPerSecondRounded();
        this.acceleration = acceleration; // full speed in 1 second
        this.defaultAcceleration = acceleration;
        log = new Logger("AccelMotor " + Utils.getMotorId(motor));
    }
    
    public void setMaxAcceleration(double acceleration)
    {
        this.acceleration = Math.abs(acceleration);
    }
    
    public void setDefaultAcceleration()
    {
        this.acceleration = defaultAcceleration;
    }
    
    public double getMaxSpped()
    {
        return maxSpeed;
    }
    
    @Override
    protected synchronized void internalSetPower(double power)
    {
        if (currentJob != null && !currentJob.isDone())
        {
            currentJob.cancel(true);
        }
        if (acceleration == Double.POSITIVE_INFINITY)
        {
            controller.setMotorPower(portNumber, power);
            return;
        }
        if (power == 0 && controller.getMotorMode(portNumber) == RunMode.RUN_TO_POSITION)
        {
            controller.setMotorPower(portNumber, 0);
            return;
        }
        double powerNow = adjustPower(getPower());
        double v0 = powerNow * maxSpeed;
        double vf = power * maxSpeed;
        if (v0 == vf) return;
        
        final long sampleTime = 15;
        double step = acceleration * (sampleTime / 1000.0) * Math.signum(vf - v0);
        log.d("Ramping power from %.3f to %.3f; step=%.3f", v0, vf, step);
        controller.setMotorPower(portNumber, step / maxSpeed); // Make PIDMotor happy
        currentJob = GlobalThreadPool.instance().start(() ->
        {
            for (double v = v0 + step; ; v += step)
            {
                if ((vf > v0 && v > vf) || (v0 >= vf && vf >= v))
                {
                    break;
                }
                log.v("power=%.3f", v / maxSpeed);
                controller.setMotorPower(portNumber, v / maxSpeed);
                try
                {
                    Thread.sleep(sampleTime);
                }
                catch (InterruptedException e)
                {
                    return;
                }
            }
            controller.setMotorPower(portNumber, vf / maxSpeed); // Make sure we're at the correct speed
        });
    }
    
    public boolean isAccelerating()
    {
        return currentJob != null && !currentJob.isDone();
    }
}
