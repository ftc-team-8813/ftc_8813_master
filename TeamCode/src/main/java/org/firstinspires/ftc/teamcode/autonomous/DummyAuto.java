package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;


@Autonomous(name="Autonomous")
public class DummyAuto extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        Drivetrain drivetrain = Robot.instance().drivetrain;
        drivetrain.drive(0.5, 0, 0);
        Thread.sleep(1500);
        drivetrain.drive(0, 0, 0);
    }
}
