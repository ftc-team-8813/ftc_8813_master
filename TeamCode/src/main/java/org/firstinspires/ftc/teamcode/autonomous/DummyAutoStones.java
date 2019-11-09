package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;


@Autonomous(name="DummyAutoStones")
public class DummyAutoStones extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();;
        robot.drivetrain.drive(0.2, 0, 0);
        Thread.sleep(3000);
        robot.drivetrain.drive(0, 0, 0);
    }
}
