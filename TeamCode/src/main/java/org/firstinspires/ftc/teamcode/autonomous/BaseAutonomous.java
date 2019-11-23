package org.firstinspires.ftc.teamcode.autonomous;
//Note: Use Ctrl+Alt+O to organize and remove unused (grayed out) imports.
//Guys, you should read ALL the comments! They contain some useful observations about the software!

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.sensors.vision.CameraStream;
import org.firstinspires.ftc.teamcode.common.sensors.vision.WebcamStream;
import org.firstinspires.ftc.teamcode.common.util.GlobalDataLogger;
import org.firstinspires.ftc.teamcode.common.util.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Persistent;
import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;
import org.opencv.android.OpenCVLoader;

import java.io.IOException;
import java.util.List;
import java.util.Vector;

/**
 * Base autonomous OpMode. Sub-OpModes that are going to be used for the games must extend this.
 * <p>
 * ## Ideas ##
 * * Have a number of OpModes and then use some selector switches on the robot for different run
 * modes (i.e. Red/Blue and 1 Jewel/2 Jewels)
 * <p>
 * ## New Programmers ##
 * * If you have any sub-methods in your OpModes/Tasks that you expect to wait or run for a long
 * time (>0.1 second), please declare them as 'throws InterruptedException'. If you're not sure,
 * just make it throw InterruptedException. This makes it so that if the robot tries to stop
 * (e.g. because the 30-second autonomous period has ended or the stop button has been pressed),
 * it stops instead of crashing with a 'The robot is stuck in stop()' message and restarting the
 * robot. **Important**: If you have a while doLoop, such as this:
 * <p>
 * while (condition) {
 * //Stuff
 * }
 * <p>
 * you need to put another condition so that it will stop when the thread is interrupted, like
 * this:
 * <p>
 * while (condition && !Thread.interrupted()) {
 * //Stuff
 * }
 * <p>
 * This way you can avoid 'stuck in stop()' errors.
 */
//The @Autonomous annotation is required for all OpModes. This puts it in the list of autonomous
//programs. You should specify a name for each OpMode.
//If you want to disable any OpModes you have, you should include the @Disabled annotation.
//If your OpMode is missing from the list of OpModes, the case is probably that you forgot to mark
//it as @Autonomous.
//This one won't be listed since it's not a functional OpMode, just a base.
// @Autonomous(name = "Autonomous")
public abstract class BaseAutonomous extends LinearOpMode
{

    //We're making BaseAutonomous a 'singleton' class. This means that there is always only ONE
    //instance in use at a time. This is stored in this static field, which can be retrieved by
    //other code without having to pass an instance to all of the methods that want to use it.
    private static BaseAutonomous instance;
    
    
    private CameraStream stream;
    private Logger log;
    public Config config;
    
    // Switch for GlobalDataLogger
    private static final boolean LOGGING_ENABLED = true;
    
    /**
     * Get the current instance of BaseAutonomous. This is set when the OpMode is initialized and
     * unset when it is stopped.
     *
     * @return An instance of BaseAutonomous
     */
    public static final BaseAutonomous instance()
    {
        return instance;
    }
    
    public static final boolean instantiated()
    {
        return instance != null;
    }
    
    public final CameraStream getCameraStream()
    {
        if (stream == null)
        {
            try
            {
                stream = new WebcamStream();
            } catch (IllegalStateException e)
            {
                stream = new CameraStream();
            }
        }
        return stream;
    }

    // Apparently, the LinearOpMode does not interrupt the current thread when it stops! This thread
    // watches the running status of the OpMode and interrupts it when a stop is requested.
    private class Interrupter implements Runnable
    {
        private List<Thread> toInterrupt;

        Interrupter(Thread toInterrupt)
        {
            this.toInterrupt = new Vector<>();
            this.toInterrupt.add(toInterrupt);
        }

        void addThread(Thread t)
        {
            toInterrupt.add(t);
        }

        @Override
        public void run()
        {
            while (!isStopRequested())
            {
                try
                {
                    Thread.sleep(10);
                }
                catch (InterruptedException e)
                {
                    break;
                }
            }
            for (Thread t : toInterrupt.toArray(new Thread[0])) t.interrupt();
        }
    }

    private Interrupter interrupter;

    public final void addThreadToInterrupt(Thread t)
    {
        if (interrupter == null)
        {
            if (log == null) return;
            log.w("Trying to add thread to interrupt while autonomous is not running");
            return;
        }
        interrupter.addThread(t);
    }
    
    /**
     * Run the op mode.
     * ## New programmers ##
     * * Declaring a method as 'final' will make it so that it cannot be overridden by the class
     * that implements it.
     *
     * @throws InterruptedException If the OpMode is trying to stop (e.g. by the stop button or 30-
     *                              second timeout)
     */
    @Override
    public final void runOpMode() throws InterruptedException
    {
        log = null;
        //Catch any InterruptedExceptions or other unchecked exceptions so that we can unset the
        //instance in case of an error.
        Throwable exc = null;
        try
        {
            //Run initialization operations here
            //Create our latest.log file
            Logger.init();
            log = new Logger("BaseAutonomous");
            //Initialize the configuration file
            config = new Config(Config.configFile);

            // Start the interrupter thread
            interrupter = new Interrupter(Thread.currentThread());
            Thread interrupterThread = new Thread(interrupter, "BaseAutonomous interrupter");
            interrupterThread.setDaemon(true);
            interrupterThread.start();
            
            // Create our global thread pool for future threads
            GlobalThreadPool.initialize(16, this);
            
            //Clear the persistent objects since this would be a new round in competition
            Persistent.clear();
            
            GlobalDataLogger.initialize("autonomous_" + getClass().getSimpleName() + ".log.gz");
            if (LOGGING_ENABLED) GlobalDataLogger.instance().start(5);
            
            //Set the current instance
            instance = this;

            Robot.initialize(hardwareMap, config);
            initialize();
            
            //Must wait for start, otherwise the robot will run as soon as it is initialized, which can
            //be incredibly annoying. We could also simply override start(), but we also want to
            //initialize stuff, so it makes it simpler to use one method.
            waitForStart();

            if (!opModeIsActive()) return;
            Logger.startTimer();
            // Run the robot code
            run();
            log.i("Finished main robot thread; waiting for OpMode to finish");
            interrupterThread.join();
        }
        catch (InterruptedException | RuntimeException | IOException | Error e)
        {
            exc = e;
        }
        finally
        {
            finish();
            if (Robot.instance() != null) Robot.instance().uninitialize();
            instance = null;
            if (stream != null)
            {
                stream.stop();
                stream = null;
            }
            if (exc != null)
            {
                //We can't just throw any Throwables; we need to throw either unchecked exceptions
                //(RuntimeException and Error) or InterruptedException, which it is declared to be
                //able to throw.
                log.e(exc);
                Logger.close();
                if (exc instanceof Error) throw (Error) exc;
                else if (exc instanceof RuntimeException) throw (RuntimeException) exc;
                else if (exc instanceof IOException) log.e(exc);
                else throw (InterruptedException) exc;
            }
            GlobalThreadPool.instance().stopAll();
            Logger.close();
            System.gc();
        }
    }
    
    /**
     * Executed when the robot is initializing. Implementation is optional; does nothing by default.
     *
     * @throws InterruptedException if the OpMode is trying to stop
     */
    public void initialize() throws InterruptedException
    {
    }
    
    /**
     * Executed when the 'play' button is pressed. Implementation is required. Main tasks should be
     * added to the list here.
     *
     * @throws InterruptedException if the OpMode is trying to stop
     */
    public abstract void run() throws InterruptedException;
    
    /**
     * Executed after all tasks have completed. Does nothing by default.
     *
     * @throws InterruptedException if the OpMode is trying to stop
     */
    public void finish() throws InterruptedException
    {
    }
    
//    /**
//     * Use this method to run all of the tasks currently in the list. This is executed after run()
//     * so that any tasks left in the queue are run. If you don't want this behavior, you should
//     * clear the task list at the end of the method. This method will remove all of the tasks from
//     * the list. Run this before any decisions about the state of the robot between tasks are made,
//     * since nothing actually happens when the tasks are added to the list.
//     *
//     * @throws InterruptedException if the OpMode is trying to stop
//     */
//    public final void runTasks() throws InterruptedException
//    {
//        while (!tasks.isEmpty())
//        {
//            //Possibly add telemetry showing which task is running?
//            Task t = tasks.remove(0);
//            t.runTask();
//        }
//    }
}
