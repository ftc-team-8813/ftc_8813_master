package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.motor_control.AccelMotor;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.GlobalDataLogger;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Profiler;
import org.firstinspires.ftc.teamcode.common.util.Scheduler;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.common.util.VMStats;
import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

import java.io.IOException;

/**
 * Main driver control
 */
public abstract class BaseTeleOp extends OpMode
{
    protected Robot robot;
    protected Logger log;
    protected double delta; // Time taken by the last tick
    
    private double prev_tick_time;
    
    private static final boolean LOGGING_ENABLED = true;
    
    @Override
    public void init()
    {
        try { Logger.init(); } catch (IOException e) { throw new RuntimeException(e); }
        log = new Logger(getClass().getCanonicalName());
        GlobalThreadPool.initialize(16);
        try
        {
            GlobalDataLogger.initialize(Config.storageDir + "teleop_" + getClass().getSimpleName() + ".log.gz");
            if (LOGGING_ENABLED) GlobalDataLogger.instance().start(5);
        } catch (IOException e)
        {
            log.e("Failed to initialize logger");
        }
        robot = Robot.initialize(hardwareMap, new Config(Config.configFile));
        prev_tick_time = (double)(System.nanoTime() / 1000000000);
        
        /*
        ((AccelMotor)robot.drivetrain.leftFront.getMotor()).setMaxAcceleration(Double.POSITIVE_INFINITY);
        ((AccelMotor)robot.drivetrain.rightFront.getMotor()).setMaxAcceleration(Double.POSITIVE_INFINITY);
        ((AccelMotor)robot.drivetrain.leftBack.getMotor()).setMaxAcceleration(Double.POSITIVE_INFINITY);
        ((AccelMotor)robot.drivetrain.rightBack.getMotor()).setMaxAcceleration(Double.POSITIVE_INFINITY);
         */
        
        
    }
    
    @Override
    public void start()
    {
        prev_tick_time = (double)System.nanoTime() / 1000000000;
    }
    
    @Override
    public final void loop()
    {
        double now = (double)System.nanoTime() / 1000000000;
        delta = now - prev_tick_time;
        doLoop();
        prev_tick_time = now;
    }

    public abstract void doLoop();

    @Override
    public void stop()
    {
        robot.uninitialize();
        GlobalDataLogger.instance().stop();
        GlobalThreadPool.instance().stopAll();
        Logger.close();
    }
}
