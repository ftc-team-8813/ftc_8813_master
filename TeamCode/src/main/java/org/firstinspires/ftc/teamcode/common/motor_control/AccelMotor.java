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
    private AccelWorker worker;
    private Logger log;
    private final double defaultAcceleration;
    private final long sampleTime = 15; // ms
    
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
        // Input power is already adjusted for direction
        if (acceleration == Double.POSITIVE_INFINITY)
        {
            controller.setMotorPower(portNumber, power);
            return;
        }
        if (power == 0 && controller.getMotorMode(portNumber) == RunMode.RUN_TO_POSITION)
        {
            if (worker != null && !currentJob.isDone())
            {
                currentJob.cancel(true);
            }
            controller.setMotorPower(portNumber, 0);
            return;
        }
        double powerNow;
        if (getMode() == RunMode.RUN_TO_POSITION) powerNow = getPower();
        else powerNow = adjustPower(getPower());
        double v0 = powerNow * maxSpeed;
        double vf = power * maxSpeed;
        log.d("%.3f --> %.3f", powerNow, power);
        if (v0 == vf) return;
        
        final long sampleTime = 15;
        double step = acceleration * (sampleTime / 1000.0) * Math.signum(vf - v0);
        controller.setMotorPower(portNumber, (v0 + step) / maxSpeed); // Make PIDMotor happy
        
        if (worker != null && !currentJob.isDone())
        {
            worker.setStep(step);
            worker.setVf(vf);
        }
        else
        {
            worker = new AccelWorker(v0, vf, step);
            currentJob = GlobalThreadPool.instance().start(worker);
        }
    }
    
    private class AccelWorker implements Runnable
    {
        private volatile  double v0, vf, step;
        
        public AccelWorker(double v0, double vf, double step)
        {
            this.v0 = v0;
            this.vf = vf;
            this.step = step;
        }
        
        public void setVf(double vf)
        {
            this.vf = vf;
        }
        
        public void setStep(double step)
        {
            this.step = step;
        }
        
        @Override
        public void run()
        {
            for (double v = v0 + step; ; v += step)
            {
                if ((step > 0 && v >= vf) || (step < 0 && vf >= v))
                {
                    break;
                }
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
        }
    }
    
    public boolean isAccelerating()
    {
        return currentJob != null && !currentJob.isDone();
    }
}
