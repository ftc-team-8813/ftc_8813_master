package org.firstinspires.ftc.teamcode.autonomous.tasks;

/**
 * A basic example Task.
 * Pauses for a certain amount of time.
 */

public class TaskDelay implements Task
{
    
    private long millis;
    
    /**
     * Create the task.
     *
     * @param millis The number of milliseconds to delay for.
     */
    public TaskDelay(long millis)
    {
        this.millis = millis;
    }
    
    @Override
    public void runTask() throws InterruptedException
    {
        //If this is interrupted by anything, it will promptly exit because that is the normal
        //behavior of Thread.sleep().
        Thread.sleep(millis);
    }
}
