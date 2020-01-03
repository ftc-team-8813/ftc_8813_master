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
            while (true)
            {
                currentValue = sensor.getDistance(DistanceUnit.MM);
                try
                {
                    Thread.sleep(7);
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
}
