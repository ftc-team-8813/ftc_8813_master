package org.firstinspires.ftc.teamcode.autonomous.tasks;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Profiler;

public class TaskIntakeMineral implements Task
{
    private Profiler profiler;

    public TaskIntakeMineral(Profiler profiler)
    {
        this.profiler = profiler;
    }

    @Override
    public void runTask() throws InterruptedException
    {
        Robot robot = Robot.instance();

        profiler.start("backup");
        // Back up
        robot.reverse(10, 0.35);
        profiler.end();

        profiler.start("drop");
        // Drop the intake
        robot.pivot.stopHolding();
        Thread.sleep(30);
        robot.intakePivot.setPower(0.5);
        Thread.sleep(200);
        robot.intakePivot.setPower(0);
        profiler.end();

        // Run the intake
        profiler.start("intake");
        robot.intake.setPower(-0.5);

        // Drive forward
        robot.forward(15, 0.35);
        Thread.sleep(100);

        // Stop the intake
        robot.intake.setPower(0);
        profiler.end();

        profiler.start("raise");
        // Raise the intake
        robot.pivot.hold(50);
        Thread.sleep(1000);
        profiler.end();

        profiler.start("put");
        // Put the mineral in the dunk bucket
        robot.intake.setPower(-0.5);
        Thread.sleep(200);
        robot.intake.setPower(0);
        profiler.end();
    }
}
