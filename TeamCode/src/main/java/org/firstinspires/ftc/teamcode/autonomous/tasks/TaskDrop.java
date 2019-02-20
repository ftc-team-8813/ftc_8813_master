package org.firstinspires.ftc.teamcode.autonomous.tasks;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Logger;

public class TaskDrop implements Task
{

    private Logger log = new Logger("TaskDrop");

    @Override
    public void runTask() throws InterruptedException
    {
        Robot robot = Robot.instance();
        robot.pivot.hold(500);
        Thread.sleep(2500);
        robot.hook.setPosition(Robot.HOOK_OPEN);
        Thread.sleep(350);
        robot.pullUp.setPower(1);
        while (!robot.pullupLimit.pressed()) Thread.sleep(5);
        robot.pullUp.setPower(0);
    }
}
