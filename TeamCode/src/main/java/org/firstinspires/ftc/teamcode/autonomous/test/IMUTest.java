package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name="IMU Test")
@Disabled
public class IMUTest extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        telemetry.addData("ERROR", "No IMU in robot!");
        telemetry.update();
//        Robot robot = Robot.instance();
//        robot.imu.initialize(telemetry);
//        robot.imu.start();

        while (opModeIsActive())
        {
            // robot.imu.update();
            Thread.sleep(50);
        }
    }
}
