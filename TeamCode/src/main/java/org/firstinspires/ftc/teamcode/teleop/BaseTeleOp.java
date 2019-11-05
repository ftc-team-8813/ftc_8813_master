package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;
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
    
    @Override
    public void init()
    {
        try { Logger.init(); } catch (IOException e) { throw new RuntimeException(e); }
        log = new Logger(getClass().getCanonicalName());
        GlobalThreadPool.initialize(4);
        robot = Robot.initialize(hardwareMap, new Config(Config.configFile));
        prev_tick_time = (double)(System.nanoTime() / 1000000000);
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
        GlobalThreadPool.instance().stopAll();
        Logger.close();
    }
}
