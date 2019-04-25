package org.firstinspires.ftc.teamcode.autonomous.tasks;

import org.firstinspires.ftc.teamcode.common.Robot;

public class TaskDunkMarker implements Task
{
    @Override
    public void runTask() throws InterruptedException
    {
        Robot robot = Robot.instance();
//        robot.dunkLiftController.hold(2 * robot.lift_up / 3); // Lifts...
//        Thread.sleep(1000);
        robot.dunk.setPosition(robot.dunk_dunk); // and dunks into the depot
        Thread.sleep(250);
        robot.dunk.setPosition(robot.dunk_min); // and then returns
        Thread.sleep(250);
//        robot.dunkLiftController.hold(0);
//        Thread.sleep(500);
    }
}
