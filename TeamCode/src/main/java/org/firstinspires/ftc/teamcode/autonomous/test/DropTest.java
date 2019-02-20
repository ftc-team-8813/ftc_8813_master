package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDrop;

@Autonomous(name="Drop Test")
public class DropTest extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        new TaskDrop().runTask();
    }
}
