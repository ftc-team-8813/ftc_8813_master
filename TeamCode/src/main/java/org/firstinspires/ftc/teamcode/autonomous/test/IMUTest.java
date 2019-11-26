package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name="IMU Test")
public class IMUTest extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();
        robot.imu.initialize();
        robot.imu.waitForInit(telemetry);
        robot.imu.start();

        while (opModeIsActive())
        {
            telemetry.addData("Heading", robot.imu.getHeading());
            telemetry.addData("Roll", robot.imu.getRoll());
            telemetry.addData("Pitch", robot.imu.getPitch());
            telemetry.update();
            Thread.sleep(50);
        }
    }
}
