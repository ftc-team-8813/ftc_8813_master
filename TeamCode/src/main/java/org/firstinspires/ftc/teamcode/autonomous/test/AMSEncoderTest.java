package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.sensors.AMSEncoder;

@Autonomous(name="AMS Encoder Test", group="test")
public class AMSEncoderTest extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        AMSEncoder encoder = Robot.instance().fwdEnc;
        AMSEncoder encoder2 = Robot.instance().strafeEnc;
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
            telemetry.addData("Fwd: Absolute angle", encoder.getAbsoluteAngle());
            telemetry.addData("Fwd: Rotations", encoder.getRotations());
            telemetry.addData("Strafe: Absolute angle", encoder2.getAbsoluteAngle());
            telemetry.addData("Strafe: Rotations", encoder2.getRotations());
            telemetry.update();

            Thread.sleep(5);
        }
    }
}
