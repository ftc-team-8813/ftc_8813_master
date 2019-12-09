package org.firstinspires.ftc.teamcode.common.util;

import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;

import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.Arrays;
import java.util.Queue;
import java.util.Scanner;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Utility to get CPU usage statistics via /system/bin/vmstat
 */
public class VMStats
{
    private volatile String lastMessage = null;
    private Future<?> thread;
    private Logger log = new Logger("VMStat");
    private Process process;
    private volatile int[] lastStats = new int[16];
    private long lastCheck;

    public static final int RUNNING       =  0;
    public static final int BLOCKED       =  1;
    public static final int SWAPPED       =  2;
    public static final int FREE_MEM      =  3;
    public static final int BUFF_MEM      =  4;
    public static final int CACHE_MEM     =  5;
    public static final int SWAP_IN       =  6;
    public static final int SWAP_OUT      =  7;
    public static final int IO_IN          = 8;
    public static final int IO_OUT        =  9;
    public static final int INTERRUPTS    = 10;
    public static final int CTXT_SWITCHES = 11;
    public static final int CPU_USER      = 12;
    public static final int CPU_SYS       = 13;
    public static final int CPU_IDLE      = 14;
    public static final int CPU_WAIT      = 15;

    public VMStats(int delay)
    {
        ProcessBuilder builder = new ProcessBuilder("/system/bin/vmstat", "-n", "" + delay);
        try
        {
            process = builder.start();
        } catch (IOException e)
        {
            log.e("VMStat failed to start");
            log.e(e);
            return;
        }

        thread = GlobalThreadPool.instance().start(() ->
        {
            Scanner scan = new Scanner(process.getInputStream());
            while (scan.hasNextLine())
            {
                lastMessage = scan.nextLine();
                try
                {
                    Thread.sleep(delay * 1000);
                }
                catch (InterruptedException e)
                {
                    scan.close();
                    return;
                }
            }
        });
        GlobalDataLogger.instance().addChannel("CPU (user)", () -> "" + getStats()[CPU_USER]);
        GlobalDataLogger.instance().addChannel("CPU (system)", () -> "" + getStats()[CPU_SYS]);
        GlobalDataLogger.instance().addChannel("Memory Free", () -> "" + getStats()[FREE_MEM]);
    }

    public synchronized int[] getStats()
    {
        // Avoid having to read multiple times from several sequential calls (i.e. for getting several statistics at once)
        if (System.currentTimeMillis() - lastCheck > 200)
        {
            lastCheck = System.currentTimeMillis();
            String stats = lastMessage;
            if (stats == null) return lastStats;
            String[] split = stats.split("\\s+");
            if (split.length < 16)
            {
                log.w("Bad vmstat output. Offending line:");
                log.w(lastMessage);
                return lastStats;
            }
            // log.d("Stats: %s", Arrays.toString(split));
            lastStats = new int[split.length-1];
            for (int i = 1; i < split.length; i++)
            {
                lastStats[i-1] = Integer.parseInt(split[i]);
            }
        }
        return lastStats;
    }

    public void close()
    {
        process.destroy();
        thread.cancel(true);
    }
}
