package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.sensors.AMSEncoder;
import org.firstinspires.ftc.teamcode.common.sensors.Odometry;
import org.firstinspires.ftc.teamcode.common.sensors.OdometryEncoder;

@Autonomous(name="AMS Encoder Test", group="test")
public class AMSEncoderTest extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        OdometryEncoder encoder = Robot.instance().fwdEnc;
        OdometryEncoder encoder2 = Robot.instance().strafeEnc;
        Robot.instance().imu.setImmediateStart(true);
        Robot.instance().imu.initialize();
        Odometry odometry = new Odometry(encoder, encoder2, Robot.instance().imu);
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
            telemetry.addData("Fwd: Position", encoder.getPosition());
            telemetry.addData("Strafe: Position", encoder2.getPosition());
            telemetry.addData("Calculated Forward", odometry.getForwardDistance());
            telemetry.addData("Calculated Strafe", odometry.getStrafeDistance());
            telemetry.update();

            Thread.sleep(5);
        }
    }
}
