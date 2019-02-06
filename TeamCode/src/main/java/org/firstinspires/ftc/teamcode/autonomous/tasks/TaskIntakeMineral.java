package org.firstinspires.ftc.teamcode.autonomous.tasks;

import org.firstinspires.ftc.teamcode.common.Robot;

public class TaskIntakeMineral implements Task
{

    @Override
    public void runTask() throws InterruptedException
    {
        Robot robot = Robot.instance();

        // Back up
        robot.reverse(15, 0.25);

        // Drop the intake
        robot.pivot.stopHolding();
        Thread.sleep(30);
        robot.intakePivot.setPower(0.5);
        Thread.sleep(400);
        robot.intakePivot.setPower(0);

        // Run the intake
        robot.intake.setPower(-0.5);

        // Drive forward
        robot.forward(15, 0.25);
        Thread.sleep(500);

        // Stop the intake
        robot.intake.setPower(0);

        // Raise the intake
        robot.pivot.hold(50);
        Thread.sleep(1000);
        robot.pivot.stopHolding();

        // Put the mineral in the dunk bucket
        robot.intake.setPower(-0.5);
        Thread.sleep(200);
        robot.intake.setPower(0);

    }
}
