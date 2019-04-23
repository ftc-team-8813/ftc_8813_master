package org.firstinspires.ftc.teamcode.autonomous.tasks;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
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
        robot.pullUp.setPower(-1);
        Thread.sleep(1300);
        robot.pullUp.setPower(0);
        long start = System.currentTimeMillis();
        while (robot.imu.getInternalImu().getGravity().xAccel > -9.79)
        {
            robot.imu.update();
//            log.d("Heading: %.4f", robot.imu.getHeading());
//            Acceleration gravity = robot.imu.getInternalImu().getGravity();
//            log.d("Gravity: <%.4f, %.4f, %.4f>", gravity.xAccel, gravity.yAccel, gravity.zAccel);
            Thread.sleep(5);
        }
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
            try
            {
                Thread.sleep(3500);
            }
            catch (InterruptedException e)
            {
            }
            robot.pullUp.setPower(0);
        }
    }
}
