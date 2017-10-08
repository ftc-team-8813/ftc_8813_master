package org.firstinspires.ftc.teamcode.autonomous;
//Note: Use Ctrl+Alt+O to organize and remove unused (grayed out) imports.
//Guys, you should read ALL the comments! They contain some useful observations about the software!
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.EventHooks;
import org.firstinspires.ftc.teamcode.autonomous.tasks.Task;
import org.firstinspires.ftc.teamcode.autonomous.util.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.opencv.android.OpenCVLoader;

import java.util.ArrayList;
import java.util.List;
//Put your hours here:
//9/10: 2:30P - 5:15P -ABY
//9/11: 6:30P - 7:30P -ABY

/*
To-Do List:
  * TODO: Make tasks using the REV robotics modules (Joe/Tarun)
  * TODO: Vision Processing (Aidan/Joe)
Note: You should put to-do list items in more specific places if possible.
 */

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
 *     robot. **Important**: If you have a while loop, such as this:
 *
 *       while (condition) {
 *           //Stuff
 *       }
 *
 *     you need to put another condition so that it will stop when the thread is interrupted, like
 *     this:
 *
 *       while (condition && !Thread.interrupted()) {
 *           //Stuff
 *       }
 *
 *     This way you can avoid more 'stuck in stop()' errors.
 */
//The @Autonomous annotation is required for all OpModes. This puts it in the list of autonomous
//programs. You should specify a name for each OpMode.
//If you want to disable any OpModes you have, you should include the @Disabled annotation.
//If your OpMode is missing from the list of OpModes, the case is probably that you forgot to mark
//it as @Autonomous.
//This one won't be listed since it's not a functional OpMode, just a base.
// @Autonomous(name = "Autonomous")
public abstract class BaseAutonomous extends LinearOpMode {

    static {
        if (!OpenCVLoader.initDebug()) {
            System.exit(0);
        }
    }

    //Queue of tasks to run
    protected final List<Task> tasks = new ArrayList<>();
    //We're making BaseAutonomous a 'singleton' class. This means that there is always only ONE
    //instance in use at a time. This is stored in this static field, which can be retrieved by
    //other code without having to pass an instance to all of the methods that want to use it.
    private static BaseAutonomous instance;

    private List<EventHooks> hooks = new ArrayList<>();

    private CameraStream stream;

    public void addEventHooks(EventHooks hook) {
        hooks.add(hook);
    }

    private void fireStopEvent() {
        for (EventHooks hook : hooks) {
            hook.stop();
        }
    }

    private void fireStartEvent() {
        for (EventHooks hook : hooks) {
            hook.resume();
        }
    }

    /**
     * Get the current instance of BaseAutonomous. This is set when the OpMode is initialized.
     * @return An instance of BaseAutonomous
     */
    public static final BaseAutonomous instance() {
        return instance;
    }
    public static final boolean instantated() {
        return instance != null;
    }

    public final CameraStream getCameraStream() {
        if (stream == null)
            stream = new CameraStream();
        return stream;
    }


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
        //Catch any InterruptedExceptions or other unchecked exceptions so that we can unset the
        //instance in case of an error.
        Throwable exc = null;
        try {
            //Run initialization operations here
            //Need to initialize the motor/sensor handlers here once they are written.

            //Clear the task list in case the robot was stopped before the list was empty
            //and the OpMode wasn't re-initialized.
            tasks.clear();

            TelemetryWrapper.init(telemetry, 0);

            //Set the current instance
            instance = this;

            initialize();
            //Fire up the camera view if necessary
            fireStartEvent();

            //Must wait for start, otherwise the robot will run as soon as it is initialized, which can
            //be incredibly annoying. We could also simply override start(), but we also want to
            //initialize stuff, so it makes it simpler to use one method.
            waitForStart();
            if (!opModeIsActive()) return;
            //Run the tasks
            run();
            //Run any leftover tasks
            runTasks();
            while (opModeIsActive()) {}
        } catch (InterruptedException | RuntimeException | Error e) {
            exc = e;
        } finally {
            instance = null;
            fireStopEvent();
            if (exc != null) {
                //We can't just throw any Throwables; we need to throw either unchecked exceptions
                //(RuntimeException and Error) or InterruptedException, which it is declared to be
                //able to throw.
                if (exc instanceof Error) throw (Error)exc;
                else if (exc instanceof RuntimeException) throw (RuntimeException)exc;
                else throw (InterruptedException)exc;
            }
        }
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
