package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.tasks.Task;

import java.util.ArrayList;
import java.util.List;
//Put your hours here:
//2:30P - 5:15P -ABY
/**
 * Base autonomous OpMode. Sub-OpModes that are going to be used for the games must extend this.
 *
 * ## Ideas ##
 *   * Have a number of OpModes and then use some selector switches on the robot for different run
 *     modes (i.e. Red/Blue and 1 Jewel/2 Jewels)
 *
 * ## New Programmers ##
 *   * If you have any sub-methods in your OpModes/Tasks that you expect to wait or run for a long
 *     time (>0.1 second), please declare them as 'throws InterruptedException'. If you're not sure,
 *     just make it throw InterruptedException. This makes it so that if the robot tries to stop
 *     (e.g. because the 30-second autonomous period has ended or the stop button has been pressed),
 *     it stops instead of crashing with a 'The robot is stuck in stop()' message and restarting the
 *     robot.
 */
//The @Autonomous annotation is required for all OpModes. This puts it in the list of autonomous
//programs. You should specify a name for each OpMode.
//If you want to disable any OpModes you have, you should include the @Disabled annotation.
//If your OpMode is missing from the list of OpModes, the case is probably that you forgot to mark
//it as @Autonomous.
//This one won't be listed since it's not a functional OpMode, just a base.
// @Autonomous(name = "Autonomous")
public abstract class BaseAutonomous extends LinearOpMode {

    //Queue of tasks to run
    protected final List<Task> tasks = new ArrayList<>();

    private static Telemetry telem;
    public static Telemetry telemetry() {return telem;}



    /**
     * Run the op mode.
     * ## New programmers ##
     *   * Declaring a method as 'final' will make it so that it cannot be overridden by the class
     *     that implements it.
     * @throws InterruptedException If the OpMode is trying to stop (e.g. by the stop button or 30-
     * second timeout)
     */
    @Override
    public final void runOpMode() throws InterruptedException {
        //Run initialization operations here
        //Need to initialize the motor/sensor handlers here once they are written.

        //Clear the task list in case the robot was stopped before the list was empty
        //and the OpMode wasn't re-initialized.
        tasks.clear();
        telem = telemetry;
        initialize();

        //Must wait for start, otherwise the robot will run as soon as it is initialized, which can
        //be incredibly annoying. We could also simply override start(), but we also want to
        //initialize stuff, so it makes it simpler to use one method. This I found out last year
        //very early in the season. -ABY
        waitForStart();
        //Run the tasks
        run();
        //Run any leftover tasks
        runTasks();
    }

    /**
     * Executed when the robot is initializing. Implementation is optional. May be needed for setup
     * operations such as checking switch state.
     * @throws InterruptedException if the OpMode is trying to stop
     */
    public void initialize() throws InterruptedException {}

    /**
     * Executed when the 'play' button is pressed. Implementation is required. Main tasks should be
     * added to the list here.
     * @throws InterruptedException if the OpMode is trying to stop
     */
    public abstract void run() throws InterruptedException;

    /**
     * Use this method to run all of the tasks currently in the list. This is executed after run()
     * so that any tasks left in the queue are run. If you don't want this behavior, you should
     * clear the task list at the end of the method. This method will remove all of the tasks from
     * the list. Run this before any decisions about the state of the robot between tasks are made,
     * since nothing actually happens when the tasks are added to the list.
     * @throws InterruptedException if the OpMode is trying to stop
     */
    public final void runTasks() throws InterruptedException {
        while (!tasks.isEmpty()) {
            //Possibly add telemetry showing which task is running?
            Task t = tasks.remove(0);
            t.runTask();
        }
    }
}
