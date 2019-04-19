package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.sensors.RangeSensor;

@TeleOp(name="Range Sensor Test")
@Disabled
public class RangeTest extends OpMode
{
    private RangeSensor sensor;

    @Override
    public void init()
    {
        sensor = new RangeSensor(hardwareMap.get(Rev2mDistanceSensor.class, "range 1"));
    }

    @Override
    public void loop()
    {
        telemetry.addData("Distance", sensor.getDistance());
    }
}
