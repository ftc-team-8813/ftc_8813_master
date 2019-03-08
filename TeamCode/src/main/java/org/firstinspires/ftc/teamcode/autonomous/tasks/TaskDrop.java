package org.firstinspires.ftc.teamcode.autonomous.tasks;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
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
        robot.pullUp.setPower(-1);
        Thread.sleep(300);
        robot.pullUp.setPower(0);
        Thread.sleep(2500);
        robot.hook.setPosition(robot.HOOK_OPEN);
        Thread.sleep(350);
        Thread respooler = new Thread(new Respooler(), "Pullup Respooler Thread");
        respooler.setDaemon(true);
        BaseAutonomous.instance().addThread(respooler);
        respooler.start();
    }

    private class Respooler implements Runnable
    {

        @Override
        public void run()
        {
            Robot robot = Robot.instance();
            robot.pullUp.setPower(-1);
            int i = 0;
            while (i < 100)
            {
                if (robot.pullupLimit.pressed())
                {
                    i++;
                }
                else
                {
                    i = 0;
                }
                try
                {
                    Thread.sleep(1);
                } catch (InterruptedException e)
                {
                    break;
                }
            }
            robot.pullUp.setPower(0);
        }
    }
}
