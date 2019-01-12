package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name="Basic Crater Auto")
public class StupidCraterAuto extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.initialize(hardwareMap, config);

        robot.leftFront.setPower(-0.75);
        robot.rightFront.setPower(-0.75);
        robot.leftRear.setPower(-0.75);
        robot.rightRear.setPower(-0.75);

        Thread.sleep(1200);

        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);

    }
}
