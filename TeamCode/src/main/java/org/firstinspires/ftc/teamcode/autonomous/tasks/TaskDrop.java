package org.firstinspires.ftc.teamcode.autonomous.tasks;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Logger;

public class TaskDrop implements Task
{

    private Logger log = new Logger("TaskDrop");

    @Override
    public void runTask() throws InterruptedException
    {
        log.d("Drop started");
        long start = System.currentTimeMillis();
        Robot robot = Robot.instance();
        // Push the robot up
        robot.pivot.hold(1600);
        Thread.sleep(1500);
        robot.pivot.stopHolding();
        Thread.sleep(500);
//        // Drop the lift
//        robot.leftDunk.setPower(-0.75);
//        robot.rightDunk.setPower(0.75);
//        Thread.sleep(500);
//        robot.leftDunk.setPower(0);
//        robot.rightDunk.setPower(0);
//        // Unhook
//        robot.dunk.setPosition(Robot.dunk_min);
//        robot.hook.setPosition(Robot.HOOK_OPEN);
//        // Raise the intake
//        robot.pivot.hold(250);
//        // Wait for the hook
//        Thread.sleep(2000);
//
//        robot.leftDunk.setPower(0.75);
//        robot.rightDunk.setPower(-0.75);
//        while (!robot.liftLimit.pressed()) Thread.sleep(1);
//        robot.leftDunk.setPower(0);
//        robot.rightDunk.setPower(0);
        log.d("Dropped from lander in " + (System.currentTimeMillis() - start) + "ms");
    }
}
