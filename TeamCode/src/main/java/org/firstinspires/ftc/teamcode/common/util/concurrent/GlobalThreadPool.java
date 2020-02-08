package org.firstinspires.ftc.teamcode.common.util.concurrent;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.util.Logger;

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
    private int taskLimit;
    private Logger log;
    
    private static GlobalThreadPool instance;
    
    private GlobalThreadPool(int nthreads, BaseAutonomous autonomous)
    {
        this(nthreads);
        Thread toInterrupt = new Thread(() ->
        {
            try
            {
                while (true)
                {
                    Thread.sleep(30000);
                }
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
        taskLimit = nthreads;
        log = new Logger("GlobalThreadPool");
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
        log.i("Attempting to start task %s (%d / %d)",
                r.getClass().getName(), getTaskCount()+1, taskLimit);
        try
        {
            Future<?> task = pool.submit(r);
            tasks.add(task);
            log.i("Successfully started task %s", r.getClass().getName());
            return task;
        }
        catch (RejectedExecutionException e)
        {
            log.w("FAILED to start task %s", r.getClass().getName());
            return null;
        }
    }
    
    public synchronized <V> Future<V> start(Callable<V> c)
    {
        log.i("Attempting to start task %s (%d / %d)",
                c.getClass().getName(), getTaskCount()+1, taskLimit);
        try
        {
            Future<V> task = pool.submit(c);
            tasks.add(task);
            log.i("Successfully started task %s", c.getClass().getName());
            return task;
        }
        catch (RejectedExecutionException e)
        {
            log.w("FAILED to start task %s", c.getClass().getName());
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
