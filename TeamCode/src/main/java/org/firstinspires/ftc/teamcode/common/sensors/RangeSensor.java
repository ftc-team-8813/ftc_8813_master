package org.firstinspires.ftc.teamcode.common.sensors;

import android.support.annotation.NonNull;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;

import java.util.concurrent.Future;

public class RangeSensor
{
    private Rev2mDistanceSensor sensor;
    private volatile double currentValue;
    private Future<?> daemon;

    public RangeSensor(@NonNull Rev2mDistanceSensor sensor)
    {
        this.sensor = sensor;
        daemon = GlobalThreadPool.instance().start(() ->
        {
            int winsize = 2;
            double[] window = new double[winsize];
            int idx = 0;
            while (true)
            {
                window[idx] = sensor.getDistance(DistanceUnit.MM);
                double avg = 0;
                for (int i = 0; i < winsize; i++)
                {
                    avg += window[i];
                }
                avg /= winsize;
                currentValue = avg;
                idx++;
                idx %= winsize;
                try
                {
                    Thread.sleep(100);
                }
                catch (InterruptedException e)
                {
                    break;
                }
            }
        });
        
    }

    // Returns distance in mm
    public double getDistance()
    {
        return currentValue;
    }
    
    public void disable()
    {
        daemon.cancel(true);
    }
}
