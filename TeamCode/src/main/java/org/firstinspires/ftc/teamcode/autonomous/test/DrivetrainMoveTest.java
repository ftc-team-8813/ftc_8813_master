package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;

@Autonomous(name="Encoder Move Test")
@Disabled
public class DrivetrainMoveTest extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        Drivetrain drivetrain = Robot.instance().drivetrain;
        drivetrain.move(0.5, 0, 0, 500);
        Thread.sleep(500);
        drivetrain.move(0, 0.5, 0, 500);
        Thread.sleep(500);
        drivetrain.move(0, 0, 0.5, 500);
        Thread.sleep(500);
        drivetrain.move(0, -0.5, 0, 500);
        Thread.sleep(500);
        drivetrain.move(-0.5, 0, 0, 500);
    }
}
