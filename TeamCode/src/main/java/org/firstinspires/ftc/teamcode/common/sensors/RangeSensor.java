package org.firstinspires.ftc.teamcode.common.sensors;

import android.support.annotation.NonNull;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RangeSensor
{
    private Rev2mDistanceSensor sensor;

    public RangeSensor(@NonNull Rev2mDistanceSensor sensor)
    {
        this.sensor = sensor;
    }

    // Returns distance in mm
    public double getDistance()
    {
        return sensor.getDistance(DistanceUnit.MM);
    }
}
