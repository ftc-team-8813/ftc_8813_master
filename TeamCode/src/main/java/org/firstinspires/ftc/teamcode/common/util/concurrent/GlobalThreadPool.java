package org.firstinspires.ftc.teamcode.common.util.concurrent;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

import java.util.List;
import java.util.Vector;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.RejectedExecutionException;

public class GlobalThreadPool
{
    private ExecutorService pool;
    private List<Future<?>> tasks;
    
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
        tasks = new Vector<>();
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
    
    
    public synchronized Future<?> start(Runnable r)
    {
        try
        {
            Future<?> task = pool.submit(r);
            tasks.add(task);
            return task;
        }
        catch (RejectedExecutionException e)
        {
            return null;
        }
    }
    
    public synchronized <V> Future<V> start(Callable<V> c)
    {
        try
        {
            Future<V> task = pool.submit(c);
            tasks.add(task);
            return task;
        }
        catch (RejectedExecutionException e)
        {
            return null;
        }
    }
    
    public synchronized int getTaskCount()
    {
        tasks.removeIf(Future::isDone);
        return tasks.size();
    }
    
    public synchronized void stopAll()
    {
        pool.shutdownNow();
    }
}
