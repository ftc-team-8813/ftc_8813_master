package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.sensors.AMSEncoder;

@Autonomous(name="AMS Encoder Test", group="test")
public class AMSEncoderTest extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        AMSEncoder encoder = new AMSEncoder(hardwareMap.i2cDeviceSynch.get("encoder"));
        if (encoder.error())
        {
            telemetry.addLine("I am error");
            telemetry.update();
            Thread.sleep(3000);
            return;
        }
        encoder.resetEncoder();
        while (opModeIsActive())
        {
            telemetry.addData("Raw angle", encoder.getRawAngle());
            telemetry.addData("Angle", encoder.getAngle());
            telemetry.addData("Absolute angle", encoder.getAbsoluteAngle());
            telemetry.addData("Rotations", encoder.getRotations());
            telemetry.update();

            Thread.sleep(5);
        }
    }
}
