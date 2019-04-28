package org.firstinspires.ftc.teamcode.autonomous.tasks;

import org.firstinspires.ftc.teamcode.common.Robot;

public class TaskDunkMarker implements Task
{
    @Override
    public void runTask() throws InterruptedException
    {
        Robot robot = Robot.instance();
        robot.dunkLiftController.hold(robot.lift_up / 6); // Lifts...
        robot.intakePivot.setPosition(robot.pivot_down);
        Thread.sleep(1000);
        robot.dunk.setPosition(robot.dunk_dunk); // and dunks into the depot
        Thread.sleep(500);
        robot.dunk.setPosition(robot.dunk_min); // and then returns
        Thread.sleep(250);
        robot.dunkLiftController.hold(0);
        robot.intakePivot.setPosition(robot.pivot_up);
        Thread.sleep(500);
    }
}
