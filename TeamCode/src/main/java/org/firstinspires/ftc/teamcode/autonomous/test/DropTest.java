package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDrop;
import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name="Drop Test")
public class DropTest extends BaseAutonomous
{
    @Override
    public void initialize() throws InterruptedException
    {
        Robot robot = Robot.instance();
        robot.hook.setPosition(robot.HOOK_CLOSED);
        robot.imu.initialize(telemetry);
        robot.imu.start();
    }

    @Override
    public void run() throws InterruptedException
    {
        new TaskDrop().runTask();
    }
}
