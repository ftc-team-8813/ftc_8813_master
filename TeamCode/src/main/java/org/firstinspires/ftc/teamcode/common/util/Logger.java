package org.firstinspires.ftc.teamcode.common.util;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.Calendar;

import static java.util.Calendar.DAY_OF_MONTH;
import static java.util.Calendar.HOUR;
import static java.util.Calendar.MINUTE;
import static java.util.Calendar.MONTH;
import static java.util.Calendar.SECOND;
import static java.util.Calendar.YEAR;

/**
 * Simple logging utility
 */

public class Logger
{
    
    public static class Level
    {
        private Level()
        {
        }
        
        public static final int NONE = Integer.MIN_VALUE;
        public static final int FATAL = 0;
        public static final int ERROR = 1;
        public static final int WARN = 2;
        public static final int INFO = 3;
        public static final int DEBUG = 4;
        public static final int VERBOSE = 5;
        public static final int ALL = Integer.MAX_VALUE;
    }
    
    private static PrintStream writer;
    private static boolean open = false;
    private static long start;
    private static boolean started = false;
    private static int maxLevel = Level.ALL;
    
    private String tag;
    
    /**
     * Initialize the logger with a new file. Closes the previous log file if one is open.
     *
     * @param file The new file to write to
     * @throws IOException If an I/O error occurs
     */
    public static void init(File file) throws IOException
    {
        started = false;
        close();
        file.getParentFile().mkdirs();
        writer = new PrintStream(file);
    }
    
    /**
     * Close the log file. Any logging operations after this will produce a NullPointerException
     * until {@link #init(File)} is called again. Does not produce {@link IOException}s.
     */
    public static void close()
    {
        if (writer != null)
        {
            writer.close();
            writer = null;
        }
    }
    
    /**
     * Set the maximum logging level to print. The default is {@link Level#ALL ALL}.
     *
     * @param level The maximum log level
     */
    public static void setLevel(int level)
    {
        maxLevel = level;
    }
    
    public static void startTimer()
    {
        start = System.currentTimeMillis();
        started = true;
    }
    
    public Logger(String tag)
    {
        this.tag = tag;
    }
    
    public synchronized void log(int level, String fmt, Object... args)
    {
        if (writer == null)
        {
            try
            {
                init(new File(Config.storageDir + "latest.log"));
            } catch (IOException e)
            {
                throw new RuntimeException(e);
            }
        }
        if (level <= maxLevel)
        {
            String base = base(level);
            writer.println(base + String.format(fmt, args));
        }
    }
    
    public synchronized void v(String fmt, Object... args)
    {
        log(99, fmt, args);
    }
    
    public synchronized void d(String fmt, Object... args)
    {
        log(4, fmt, args);
    }
    
    public synchronized void i(String fmt, Object... args)
    {
        log(3, fmt, args);
    }
    
    public synchronized void w(String fmt, Object... args)
    {
        log(2, fmt, args);
    }
    
    public synchronized void e(String fmt, Object... args)
    {
        log(1, fmt, args);
    }
    
    public synchronized void e(Throwable t)
    {
        if (writer == null)
        {
            try
            {
                init(new File(Config.storageDir + "latest.log"));
            } catch (IOException e)
            {
                throw new RuntimeException(e);
            }
        }
        if (1 <= maxLevel)
        {
            String base = base(1);
            writer.print(base);
            t.printStackTrace(writer);
        }
    }
    
    private String base(int level)
    {
        String lvl;
        if (level <= 0) lvl = "FATAL";
        else if (level == 1) lvl = "ERROR";
        else if (level == 2) lvl = "WARN";
        else if (level == 3) lvl = "INFO";
        else if (level == 4) lvl = "DEBUG";
        else lvl = "VERBOSE";
        Calendar c = Calendar.getInstance();
        //                     year  mo   dy  hour min  sec tg lv
        if (started)
        {
            long secs = (System.currentTimeMillis() - start) / 1000;
            return String.format("%04d/%02d/%02d %02d:%02d:%02d [%ds] %s/%s: ",
                    c.get(YEAR), c.get(MONTH) + 1, c.get(DAY_OF_MONTH), c.get(HOUR), c.get(MINUTE),
                    c.get(SECOND), secs, tag, lvl);
        }
        return String.format("%04d/%02d/%02d %02d:%02d:%02d %s/%s: ",
                c.get(YEAR), c.get(MONTH) + 1, c.get(DAY_OF_MONTH), c.get(HOUR), c.get(MINUTE),
                c.get(SECOND), tag, lvl);
    }
    
    public synchronized void f(String fmt, Object... args)
    {
        log(0, fmt, args);
    }
}
