package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name="Basic Autonomous")
public class StupidAutonomous extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.initialize(hardwareMap, config);

        robot.leftFront.setPower(-0.5);
        robot.rightFront.setPower(-0.5);
        robot.leftRear.setPower(-0.5);
        robot.rightRear.setPower(-0.5);

        Thread.sleep(800);

        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);

        robot.dunk.setPosition(0.2);
        Thread.sleep(1000);
        robot.dunk.setPosition(0);
    }
}
