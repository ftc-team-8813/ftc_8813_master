package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.sensors.RangeSensor;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

@TeleOp(name="Range Sensor Test")
public class RangeTest extends BaseTeleOp
{
    @Override
    public void init()
    {
        super.init();
    }
    
    @Override
    public void doLoop()
    {
        telemetry.addData("L range", robot.leftRange.getDistance());
        telemetry.addData("C range", robot.centerRange.getDistance());
        telemetry.addData("R range", robot.rightRange.getDistance());
    }
}
