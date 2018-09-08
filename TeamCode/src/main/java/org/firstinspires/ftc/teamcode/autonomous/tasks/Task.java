package org.firstinspires.ftc.teamcode.autonomous.tasks;

/**
 * A basic task.
 * Implement Tasks and override their runTask() method to make individual tasks that can be queued
 * by implementations of BaseAutonomous. All Task names should be prefixed with 'Task' and should be
 * in the 'autonomous.tasks' package.
 */

public interface Task
{
    /**
     * Run the task.
     *
     * @throws InterruptedException if the OpMode is trying to stop
     */
    public void runTask() throws InterruptedException;
}
