package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;


@Autonomous(name="TestAuto")
public class DummyAuto extends BaseAutonomous
{
    Robot robot = Robot.instance();


    @Override
    public void run() throws InterruptedException
    {
        robot.foundationHook.moveHookUp();
        robot.drivetrain.drive(1, 0, 0);
        Thread.sleep(1000);
        robot.drivetrain.drive(0, 0, 0);

        robot.drivetrain.drive(0, 1, 0);
        Thread.sleep(1000);
        robot.drivetrain.drive(0, 0, 0);
        robot.foundationHook.moveHookDown();
    }
}
