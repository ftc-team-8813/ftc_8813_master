package org.firstinspires.ftc.teamcode.common.util.concurrent;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.RejectedExecutionException;

public class GlobalThreadPool
{
    private ExecutorService pool;
    
    private static GlobalThreadPool instance;
    
    private GlobalThreadPool(int nthreads, BaseAutonomous autonomous)
    {
        this(nthreads);
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
        try
        {
            return pool.submit(r);
        }
        catch (RejectedExecutionException e)
        {
            return null;
        }
    }
    
    public <V> Future<V> start(Callable<V> c)
    {
        try
        {
            return pool.submit(c);
        }
        catch (RejectedExecutionException e)
        {
            return null;
        }
    }
    
    public void stopAll()
    {
        pool.shutdownNow();
    }
}
