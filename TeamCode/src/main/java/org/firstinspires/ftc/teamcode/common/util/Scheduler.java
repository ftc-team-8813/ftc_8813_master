package org.firstinspires.ftc.teamcode.common.util;

import java.util.ArrayList;
import java.util.List;

// WARNING: This code is currently NOT thread-safe
public class Scheduler
{

    private Logger log = new Logger("Scheduler");

    private class Task
    {
        String name;
        long start;
        long delay;
        boolean repeat;
        Runnable task;
    }

    public class TaskCallback
    {
        private final Task task;
        TaskCallback(Task t)
        {
            task = t;
        }

        public void run()
        {
            log.d("Running task " + task.name + " on request");
            task.task.run();
            tasks.remove(task);
        }

        public void cancel()
        {
            log.d("Cancelled task " + task.name);
            tasks.remove(task);
        }
    }

    private List<Task> tasks = new ArrayList<>();

    public TaskCallback add(String name, long delay, Runnable task, boolean repeat)
    {
        Task t = new Task();
        t.name = name;
        t.start = System.currentTimeMillis();
        t.delay = delay;
        t.task = task;
        t.repeat = repeat;
        tasks.add(t);
        return new TaskCallback(t);
    }

    public TaskCallback add(long delay, Runnable task, boolean repeat)
    {
        return add("Task-" + tasks.size(), delay, task, repeat);
    }

    public TaskCallback add(String name, long delay, Runnable task)
    {
        return add(name, delay, task, false);
    }

    public TaskCallback add(long delay, Runnable task)
    {
        return add(delay, task, false);
    }

    public void update()
    {
        for (int i = 0; i < tasks.size(); i++)
        {
            Task t = tasks.get(i);
            log.d("Running task " + t.name);
            if (System.currentTimeMillis() >= t.start + t.delay)
            {

                t.task.run();
                if (t.repeat)
                {
                    t.start = System.currentTimeMillis();
                    log.d("Repeating task in " + t.delay + "ms");
                }
                else
                {
                    tasks.remove(t);
                    i--;
                }
            }
        }
    }
}
