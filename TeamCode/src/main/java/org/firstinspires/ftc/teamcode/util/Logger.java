package org.firstinspires.ftc.teamcode.util;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.Calendar;
import static java.util.Calendar.*;

/**
 * Simple logging utility
 */

public class Logger {

    public static class Level {
        public static final int NONE    = Integer.MIN_VALUE;
        public static final int FATAL   = 0;
        public static final int ERROR   = 1;
        public static final int WARN    = 2;
        public static final int INFO    = 3;
        public static final int DEBUG   = 4;
        public static final int VERBOSE = 5;
        public static final int ALL     = Integer.MAX_VALUE;
    }

    private static PrintStream writer;
    private static boolean open = false;
    private static int maxLevel = Level.ALL;

    private String tag;

    /**
     * Initialize the logger with a new file. Closes the previous log file if one is open.
     * @param file The new file to write to
     * @throws IOException If an I/O error occurs
     */
    public static void init(File file) throws IOException {
        close();
        writer = new PrintStream(file);
    }

    /**
     * Close the log file. Any logging operations after this will produce a NullPointerException
     * until {@link #init(File)} is called again. Does not produce {@link IOException}s.
     */
    public static void close() {
        if (writer != null) {
            writer.close();
            writer = null;
        }
    }

    /**
     * Set the maximum logging level to print
     * @param level
     */
    public static void setLevel(int level) {
        maxLevel = level;
    }

    public Logger(String tag) {
        this.tag = tag;
    }

    public void log(int level, String fmt, Object... args) {
        if (writer == null) {
            try {
                init(new File(Config.storageDir + "latest.log"));
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
        if (level <= maxLevel) {
            String lvl;
            if (level <= 0) lvl = "FATAL";
            else if (level == 1) lvl = "ERROR";
            else if (level == 2) lvl = "WARN";
            else if (level == 3) lvl = "INFO";
            else if (level == 4) lvl = "DEBUG";
            else lvl = "VERBOSE";
            Calendar c = Calendar.getInstance();
            //                           year  mo   dy  hour min  sec
            String base = String.format("%04d/%02d/%02d %02d:%02d:%02d %s/%s: ",
                    c.get(YEAR), c.get(MONTH)+1, c.get(DAY_OF_MONTH), c.get(HOUR), c.get(MINUTE),
                    c.get(SECOND), tag, lvl);
            writer.println(base + String.format(fmt, args));
        }
    }

    public synchronized void v(String fmt, Object... args) {
        log(99, fmt, args);
    }

    public synchronized void d(String fmt, Object... args) {
        log(4, fmt, args);
    }

    public synchronized void i(String fmt, Object... args) {
        log(3,fmt, args);
    }

    public synchronized void w(String fmt, Object... args) {
        log(2, fmt, args);
    }

    public synchronized void e(String fmt, Object... args) {
        log(1, fmt, args);
    }

    public synchronized void e(Throwable t) {
        if (writer == null) {
            try {
                init(new File(Config.storageDir + "latest.log"));
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
        if (1 <= maxLevel) {
            String lvl = "ERROR";
            Calendar c = Calendar.getInstance();
            //                           year  mo   dy  hour min  sec
            String base = String.format("%04d/%02d/%02d %02d:%02d:%02d %s/%s: ",
                    c.get(YEAR), c.get(MONTH)+1, c.get(DAY_OF_MONTH), c.get(HOUR), c.get(MINUTE),
                    c.get(SECOND), tag, lvl);
            writer.print(base);
            t.printStackTrace(writer);
        }
    }

    public synchronized void f(String fmt, Object... args) {
        log(0, fmt, args);
    }
}
