package org.firstinspires.ftc.teamcode.autonomous.tasks;

/**
 * A basic task.
 * Implement Tasks and override their runTask() method to make individual tasks that can be queued
 * by implementations of BaseAutonomous.
 */

public interface Task {
    public void runTask() throws InterruptedException;
}
