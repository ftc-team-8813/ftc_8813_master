package org.firstinspires.ftc.teamcode.common.util.concurrent;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class GlobalThreadPool
{
    private ExecutorService pool;
    private BaseAutonomous autonomous;
    
    private static GlobalThreadPool instance;
    
    private GlobalThreadPool(int nthreads, BaseAutonomous autonomous)
    {
        this(nthreads);
        this.autonomous = autonomous;
        Thread toInterrupt = new Thread(() ->
        {
            try
            {
                Thread.sleep(30000);
            }
            catch (InterruptedException e) {}
            finally
            {
                pool.shutdownNow(); // Interrupt everything
            }
        });
        toInterrupt.setDaemon(true);
        toInterrupt.start();
        autonomous.addThreadToInterrupt(toInterrupt);
    }
    
    private GlobalThreadPool(int nthreads)
    {
        pool = Executors.newFixedThreadPool(nthreads);
    }
    
    public static GlobalThreadPool initialize(int nthreads, BaseAutonomous auton)
    {
        instance = new GlobalThreadPool(nthreads, auton);
        return instance;
    }
    
    public static GlobalThreadPool initialize(int nthreads)
    {
        instance = new GlobalThreadPool(nthreads);
        return instance;
    }
    
    public static GlobalThreadPool instance()
    {
        return instance;
    }
    
    
    public Future<?> start(Runnable r)
    {
        return pool.submit(r);
    }
    
    public <V> Future<V> start(Callable<V> c)
    {
        return pool.submit(c);
    }
}
