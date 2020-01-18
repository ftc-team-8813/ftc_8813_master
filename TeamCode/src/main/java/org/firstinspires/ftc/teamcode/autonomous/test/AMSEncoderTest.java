package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.sensors.AMSEncoder;
import org.firstinspires.ftc.teamcode.common.sensors.OdometryEncoder;

@Autonomous(name="AMS Encoder Test", group="test")
public class AMSEncoderTest extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        OdometryEncoder encoder = Robot.instance().fwdEnc;
        OdometryEncoder encoder2 = Robot.instance().strafeEnc;
        if (encoder == null || encoder2 == null)
        {
            telemetry.addLine("Encoder not connected!");
            telemetry.update();
            Thread.sleep(3000);
            return;
        }
        if (encoder.error() || encoder2.error())
        {
            telemetry.addLine("I am error");
            telemetry.update();
            Thread.sleep(3000);
            return;
        }
        encoder.resetEncoder();
        while (opModeIsActive())
        {
            telemetry.addData("Fwd: Absolute angle", encoder.getPosition());
            telemetry.addData("Strafe: Absolute angle", encoder2.getPosition());
            telemetry.update();

            Thread.sleep(5);
        }
    }
}
